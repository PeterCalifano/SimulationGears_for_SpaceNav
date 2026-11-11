function [varImage, strDiagnostics, varRawImage, strSurfaceLabels, fLinearDetectorImage] = ...
        RenderCameraFrame(objRequest, objRenderer, options)
%% SIGNATURE
% [varImage, strDiagnostics, varRawImage, strSurfaceLabels, fLinearDetectorImage] = ...
%     RenderCameraFrame(objRequest, objRenderer, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Render one grayscale acquisition using an injected Spectra or Blender renderer.
% Spectra geometry is target-body-fixed; Blender receives explicit world geometry.
% The caller owns renderer lifetime, RNG policy, output persistence and scheduling.
% Retain the legacy one-DN floor and normalized Blender intensity conversion.
% Spectra's Eigen/gtwrap getter transports packed uint32 pixel words as doubles;
% validate their integer range before conversion, without changing channel order.
% This host adapter has no Nav-Frontend, backend, or simulation-state dependency.
% Reject native detector providers before applying hosted stochastic image errors.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objRequest  SImageRequest with positions in the renderer's mesh length unit.
% objRenderer Spectra protocol (setLightPosition, setCameraPose, renderFrame,
%             getImageRGBA), or Blender renderImage protocol. Mutated in place.
% options     Blender quaternion/port settings and opt-in same-render dense labels.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% varImage             Nonempty numeric grayscale image in digital numbers [0,255].
% strDiagnostics       Original timestamp/frame identity and the metadata fields
%                      bHasLinearDetectorImage, dReadNoiseSigmaNorm and
%                      dDetectorNormalizationLevels. It stores no image payload.
% varRawImage          Grayscale image before the configured image-error pipeline.
% strSurfaceLabels     Optional dense surface labels from the same render; empty struct when disabled.
% fLinearDetectorImage Signed single-precision normalized processing signal from
%                      the same acquisition and noise draw; empty when later
%                      nonlinear, bloom, dead-pixel, or gamma effects are active.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-10-2026  Pietro Califano, Codex gpt-6.1  Move renderer boundary to its owner.
% 18-10-2026  Pietro Califano, Codex GPT-6  Capture optional dense labels from the same acquisition.
% 18-10-2026  Pietro Califano, Codex gpt-6.1  Guard hosted stochastic noise ownership.
% 19-10-2026  Pietro Califano, Codex GPT-6  Report detector signal normalization levels.
% 19-10-2026  Pietro Califano, Codex GPT-6  Expose signed detector readout from one acquisition.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; image prefixes match the varying class.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Decode packed RGBA locally; name the Blender frame as text.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Image Processing Toolbox; a Spectra or Blender renderer object supplied by the caller.
% See RunCameraFrameExample for executable use and representative output.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    objRequest (1,1) SImageRequest
    objRenderer (1,1)
    options.bConvertCamQuatToBlenderQuat (1,1) logical = true
    options.ui32TargetPort (1,1) uint32 = uint32(0)
    options.bCaptureSurfaceLabels (1,1) logical = false
end
arguments (Output)
    varImage (:,:) {mustBeNumeric, mustBeReal, mustBeFinite}
    strDiagnostics (1,1) struct
    varRawImage (:,:) {mustBeNumeric, mustBeReal, mustBeFinite}
    strSurfaceLabels (1,1) struct
    fLinearDetectorImage (:,:) single {mustBeReal, mustBeFinite}
end

% Initialize the values that only some branches assign
strSurfaceLabels = struct();
dReadNoiseSigmaNorm = 0;

if options.bCaptureSurfaceLabels && ~ismethod(objRenderer, 'renderFrameWithLabels')
    error('RenderCameraFrame:SurfaceLabelsUnavailable', ...
        'Dense surface labels require the Spectra renderFrameWithLabels interface.');
end

% Inspect capabilities without sampling a frame or changing native detector state.
AssertHostedCameraNoiseCompatible(objRequest.objImageErrorSettings, objRenderer);

% Use the concrete renderer protocol without introducing a second renderer hierarchy.
if ismethod(objRenderer, 'setLightPosition') && ismethod(objRenderer, 'setCameraPose') && ...
        ismethod(objRenderer, 'renderFrame') && ismethod(objRenderer, 'getImageRGBA')

    % Express the Sun and camera in the target-body frame
    dDCM_TBfromW = objRequest.dDCM_WfromTB';
    dCameraPosition_TB = dDCM_TBfromW * ...
        (objRequest.dCameraPosition_W - objRequest.dTargetPosition_W);
    dSunPosition_TB = dDCM_TBfromW * ...
        (objRequest.dSunPosition_W - objRequest.dTargetPosition_W);

    objRenderer.setLightPosition(dSunPosition_TB);
    objRenderer.setCameraPose(dCameraPosition_TB, dDCM_TBfromW * objRequest.dDCM_WfromCAM);

    % Existing renderer filenames use timestamp IDs, not the tracker's frame IDs.
    if options.bCaptureSurfaceLabels
        if objRequest.i32FrameID < 0
            error('RenderCameraFrame:InvalidLabelFrame', ...
                'Dense labels require a nonnegative acquisition frame ID.');
        end

        strCaptureInfo = struct('scene_revision', uint64(0), ...
            'camera_revision', uint64(objRequest.ui32TimestampID), ...
            'acquisition_id', sprintf('%u', objRequest.ui32TimestampID), ...
            'frame_id', sprintf('%d', objRequest.i32FrameID), ...
            'epoch_s', objRequest.dTimestamp, ...
            'epoch_reference', 'simulation time', ...
            'world_frame', 'TARGET_BODY', ...
            'camera_frame', 'CAMERA', ...
            'world_unit_m', objRequest.dLengthUnit_m);

        strSurfaceLabels = objRenderer.renderFrameWithLabels(strCaptureInfo, objRequest.ui32TimestampID);
    else
        objRenderer.renderFrame(false, objRequest.ui32TimestampID);
    end

    % Read the packed RGBA words and check their transport class and shape
    varPackedImage = objRenderer.getImageRGBA();
    if ~(isa(varPackedImage, 'uint32') || isa(varPackedImage, 'double')) || ...
            ~ismatrix(varPackedImage) || ~isreal(varPackedImage) || isempty(varPackedImage)
        error('RenderCameraFrame:InvalidRendererImage', ...
            'Spectra must return a nonempty H-by-W packed RGBA image as uint32 or double words.');
    end

    % Preserve the existing wrapper transport, rejecting invalid narrowing
    % rather than silently rounding fractional or saturating overflowing words.
    if isa(varPackedImage, 'double') && ...
            (any(~isfinite(varPackedImage), 'all') || any(varPackedImage < 0, 'all') || ...
             any(varPackedImage > double(intmax('uint32')), 'all') || ...
             any(varPackedImage ~= fix(varPackedImage), 'all'))
        error('RenderCameraFrame:InvalidRendererImage', ...
            'Packed RGBA doubles must be finite integers in the uint32 range.');
    end

    % Split the packed RGBA8 words into RGB: red is the lowest byte, then green and blue; drop alpha
    ui32PackedImage = uint32(varPackedImage);
    ui8ImageRGB = cat(3, uint8(bitand(ui32PackedImage, 255)), ...
                         uint8(bitand(bitshift(ui32PackedImage, -8), 255)), ...
                         uint8(bitand(bitshift(ui32PackedImage, -16), 255)));
    varImage = im2gray(ui8ImageRGB);

elseif ismethod(objRenderer, 'renderImage')

    % Connect the Blender transport on first use
    if isprop(objRenderer, 'bCommManagerReady') && ~objRenderer.bCommManagerReady
        objRenderer.Initialize();
    end

    % Name the frame as text so this file does not need CommManager4MATLAB's
    % EnumRenderingFrame; BlenderPyCommManager converts the text to its enum.
    varImage = objRenderer.renderImage(objRequest.dSunPosition_W, ...
        objRequest.dCameraPosition_W, ...
        objRequest.dDCM_WfromCAM, ...
        objRequest.dTargetPosition_W, ...
        objRequest.dDCM_WfromTB, ...
        'ui32TargetPort', options.ui32TargetPort, ...
        'enumRenderingFrame', 'CUSTOM_FRAME', ...
        'bConvertCamQuatToBlenderQuat', options.bConvertCamQuatToBlenderQuat);
else
    error('RenderCameraFrame:UnsupportedRenderer', ...
        'Supply a Spectra camera renderer or Blender renderImage provider.');
end

% Validate transport output before storing or forwarding it as an observation.
if ~isnumeric(varImage) || ~isreal(varImage) || isempty(varImage) || ...
        any(~isfinite(varImage), 'all') || any(varImage < 0, 'all')
    error('RenderCameraFrame:InvalidRendererImage', 'Renderer output must be finite nonnegative numeric pixels.');
end

if ~ismatrix(varImage)
    varImage = im2gray(varImage);
end

% Apply the one-DN floor; scale normalized float renders to 0..255 DN.
% On the normalized scale the floor is 1.01/255 = 0.003960784313725.
if isinteger(varImage) || any(varImage > 2, 'all')
    varImage(varImage <= 1) = 0;
else
    varImage(varImage <= 0.003960784313725) = 0;
    varImage = 255*varImage;
end
varRawImage = varImage;

% Return the clean readout, or apply the configured image-error pipeline
objSettings = objRequest.objImageErrorSettings;
if objSettings.bDefaultConstructed

    % Float renders use the adapter's fixed 0..255-DN convention; automatic
    % integer levels use the raw pixel type's storage maximum.
    dDetectorNormalizationLevels = objSettings.dNUM_LEVELS;
    if dDetectorNormalizationLevels == 0
        if isinteger(varRawImage)
            dDetectorNormalizationLevels = double(intmax(class(varRawImage)));
        else
            dDetectorNormalizationLevels = 255;
        end
    end

    if ~isfinite(dDetectorNormalizationLevels) || dDetectorNormalizationLevels <= 0
        error('RenderCameraFrame:InvalidDetectorNormalizationLevels', ...
            'Clean readout requires a positive finite detector normalization level.');
    end

    fLinearDetectorImage = single(double(varRawImage) ./ dDetectorNormalizationLevels);
    bHasLinearDetectorImage = true;
else
    [varImage, fLinearDetectorImage, dReadNoiseSigmaNorm, ...
        dDetectorNormalizationLevels] = ...
        ApplyCameraImageErrors(varRawImage, objSettings);

    bHasLinearDetectorImage = ~HasLaterNonlinearEffects_(objSettings);
    if ~bHasLinearDetectorImage
        fLinearDetectorImage = single([]);
    end
end

% Assemble the scalar diagnostics
strDiagnostics = struct('ui32TimestampID', objRequest.ui32TimestampID, ...
    'i32FrameID', objRequest.i32FrameID, ...
    'dTimestamp', objRequest.dTimestamp, ...
    'bHasLinearDetectorImage', bHasLinearDetectorImage, ...
    'dReadNoiseSigmaNorm', dReadNoiseSigmaNorm, ...
    'dDetectorNormalizationLevels', dDetectorNormalizationLevels);
end

function bActive = HasLaterNonlinearEffects_(objSettings)
%% SIGNATURE
% bActive = HasLaterNonlinearEffects_(objSettings)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Identify enabled effects that modify the captured detector signal downstream.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objSettings  Hosted image-error settings.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bActive      True when dead pixels or columns, gamma, bloom or bright spots are enabled.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    objSettings (1,1) SImageErrorsPipelineSettings
end

bActive = objSettings.dDeadPixelPerc > 0 || objSettings.dDeadBucketFraction > 0 || ...
    objSettings.dGammaValue > 0 || ...
    (objSettings.dBloomingProbability > 0 && objSettings.dBloomPoissonMean > 0) || ...
    (objSettings.dBrightSpotsProbability > 0 && objSettings.dBrightSpotsPoissonRate > 0);
end
