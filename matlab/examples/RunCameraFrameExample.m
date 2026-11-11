function [varImage, strDiagnostics, fLinearDetectorImage] = RunCameraFrameExample(objRenderer)
%% SIGNATURE
% [varImage, strDiagnostics, fLinearDetectorImage] = RunCameraFrameExample(objRenderer)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Render a caller-configured mesh with camera ten mesh units above its origin.
% The injected renderer owns camera intrinsics, mesh, lights and allocation.
% With the test CCameraRendererFixture, output is a 4-by-5 uint8 image of 128 DN.
% The third output is the same clean acquisition normalized to single precision.
% Example: [varImage, strInfo, fSignal] = RunCameraFrameExample(CCameraRendererFixture());
% Output: Camera frame 0 at timestamp 1: 4 x 5 pixels; fSignal is 128/255.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objRenderer  Initialized Spectra/Blender camera renderer, or test fixture.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% varImage               Numeric image in digital numbers.
% strDiagnostics         Acquisition identity and scalar signal availability/noise.
% fLinearDetectorImage   Signed normalized single-precision detector signal.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-10-2026  Pietro Califano, Codex gpt-6.1  Document injected rendering.
% 19-10-2026  Pietro Califano, Codex GPT-6  Document the optional linear detector signal.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SImageRequest, RenderCameraFrame
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    objRenderer (1,1)
end
arguments (Output)
    varImage (:,:) {mustBeNumeric}
    strDiagnostics (1,1) struct
    fLinearDetectorImage (:,:) single {mustBeReal, mustBeFinite}
end

% Place the camera on the target +Z axis and the Sun on its +X axis
objRequest = SImageRequest(uint32(1), int32(0), 0, [100; 0; 0], [0; 0; 10], eye(3), zeros(3, 1), eye(3));

% Render once and report the acquisition identity
[varImage, strDiagnostics, ~, ~, fLinearDetectorImage] = RenderCameraFrame(objRequest, objRenderer);
fprintf('Camera frame %d at timestamp %u: %d x %d pixels\n', ...
    strDiagnostics.i32FrameID, strDiagnostics.ui32TimestampID, size(varImage, 1), size(varImage, 2));
end
