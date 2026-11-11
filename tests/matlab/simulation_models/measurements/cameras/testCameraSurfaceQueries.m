function objTests = testCameraSurfaceQueries()
%% SIGNATURE
% objTests = testCameraSurfaceQueries()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Exercise real selected Spectra MEX queries and dense acquisitions on an analytic finite plane.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None. Set SIM_GEARS_SPECTRA_RT_DIR before SetupSimGears; without it the renderer tests are skipped.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests    Function-based MATLAB tests using disposable, asset-independent scenes.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-10-2026  Pietro Califano, Codex GPT-6  Qualify acquisition geometry, units and exact surface IDs.
% 19-10-2026  Pietro Califano, Codex GPT-6  Preserve label output position with linear readout.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; typed loop counters; scene file via yaml.dumpFile.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Skip the renderer tests when no capable Spectra build is set.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% QueryCameraSurface, RenderCameraFrame, spectra_rt.CSpectralRayTracerWrapper, yaml.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testMetricPointsAgreeAcrossTransformedAcquisitionUnits(testCase)
% Metre and kilometre scenes give the same metric hits after the pose was moved away.
RequireSpectraSurfaceQueries_(testCase);
cellHits = cell(1, 2);
for ui32CaseIdx = uint32(1):uint32(2)
    dUnit_m = [1, 1000];
    [objRenderer, objCamera, objRequest, objCleanup] = BuildScene_(dUnit_m(ui32CaseIdx), true); %#ok<ASGLU>
    dPixels_px = [48.5, 53.5, 48.5, 1, NaN; 32.5, 32.5, 37.5, 1, 10];

    % Deliberately move mutable renderer state away from the acquisition pose.
    objRenderer.setCameraPose([9; 8; 7]/dUnit_m(ui32CaseIdx), eye(3));
    strHits = QueryCameraSurface(objRequest, objRenderer, objCamera, dPixels_px);

    verifyEqual(testCase, strHits.status, uint32([1; 1; 1; 0; 2]));
    verifyEqual(testCase, strHits.position_m(1:3, :), ...
        [0, 0, 0; 1/3, 0, 0; 0, 1/4, 0], 'AbsTol', 2e-6);
    verifyEqual(testCase, strHits.range_m(1:3), ...
        sqrt(9 + [0; (1/3)^2; (1/4)^2]), 'AbsTol', 2e-6);
    verifyClass(testCase, strHits.class_id, 'uint64');
    verifyClass(testCase, strHits.field_validity, 'uint32');
    verifyEqual(testCase, strHits.class_id(1:3), repmat(bitshift(uint64(1), 53)+uint64(17), 3, 1));
    verifyEqual(testCase, strHits.component_id(1:3), zeros(3, 1, 'uint64'));
    verifyEqual(testCase, strHits.field_validity(1:3), repmat(uint32(7), 3, 1));
    verifyFalse(testCase, any(strHits.triangle_valid(4:5)));
    verifyEqual(testCase, strHits.info.epoch_s, objRequest.dTimestamp);
    verifyEqual(testCase, strHits.info.world_frame, 'TARGET_BODY');
    verifyEqual(testCase, strHits.info.world_unit_m, dUnit_m(ui32CaseIdx));
    cellHits{ui32CaseIdx} = strHits;
    clear objCleanup
end

verifyEqual(testCase, cellHits{1}.position_m(1:3, :), cellHits{2}.position_m(1:3, :), ...
    'AbsTol', 2e-6);
end

function testDenseAcquisitionLabelsMatchSparsePixelCentres(testCase)
% Dense labels from the render match sparse queries at every pixel centre and the analytic plane.
RequireSpectraSurfaceQueries_(testCase);
[objRenderer, objCamera, objRequest, objCleanup] = BuildScene_(1, true); %#ok<ASGLU>
[ui8Image, strIdentity, ui8RawImage, strLabels, fLinearDetectorImage] = ...
    RenderCameraFrame(objRequest, objRenderer, ...
    'bCaptureSurfaceLabels', true);

[dColumns_px, dRows_px] = meshgrid(1:96, 1:64);
strHits = QueryCameraSurface(objRequest, objRenderer, objCamera, [dColumns_px(:)'; dRows_px(:)']);
bAnalyticHit = abs(3*(dColumns_px - 48.5)/45) <= 1 & ...
    abs(3*(dRows_px - 32.5)/60) <= .6;
bSparseHit = reshape(strHits.status == uint32(1), 64, 96);

verifySize(testCase, ui8Image, [64, 96]);
verifyEqual(testCase, ui8RawImage, ui8Image);
verifyClass(testCase, fLinearDetectorImage, 'single');
verifyEqual(testCase, fLinearDetectorImage, single(double(ui8Image)/255));
verifyTrue(testCase, strIdentity.bHasLinearDetectorImage);
verifyEqual(testCase, strIdentity.dReadNoiseSigmaNorm, 0);
verifyGreaterThan(testCase, max(ui8Image, [], 'all'), 0);
verifyEqual(testCase, bSparseHit, bAnalyticHit);
verifyEqual(testCase, strLabels.status, uint32(bSparseHit)*uint32(2));
verifyEqual(testCase, strLabels.class_id, reshape(strHits.class_id, 64, 96));
verifyEqual(testCase, strLabels.component_id, reshape(strHits.component_id, 64, 96));
verifyEqual(testCase, strLabels.body_instance_id, reshape(strHits.body_instance_id, 64, 96));
verifyEqual(testCase, strLabels.field_validity(bSparseHit), repmat(uint32(7), nnz(bSparseHit), 1));
verifyEqual(testCase, strLabels.info.acquisition_id, sprintf('%u', objRequest.ui32TimestampID));
verifyEqual(testCase, strLabels.info.frame_id, sprintf('%d', objRequest.i32FrameID));
verifyEqual(testCase, strLabels.info.epoch_s, strIdentity.dTimestamp);
verifyEqual(testCase, strLabels.sampling, 'pixel_center');
verifyEqual(testCase, strLabels.row_origin, 'negative_viewport_v');
end

function testAsymmetricImageSupportMatchesLabelsAndQueries(testCase)
% An off-centre plane appears at the same pixels in the image, the labels and the queries.
RequireSpectraSurfaceQueries_(testCase);
[objRenderer, objCamera, objRequest, objCleanup] = BuildScene_(1, true, true); %#ok<ASGLU>
[~, ~, ui8RawImage, strLabels] = RenderCameraFrame(objRequest, objRenderer, ...
    'bCaptureSurfaceLabels', true);

[dColumns_px, dRows_px] = meshgrid(1:96, 1:64);
dSurfaceX_m = 3*(dColumns_px - 48.5)/45;
dSurfaceY_m = 3*(dRows_px - 32.5)/60;
bExpectedHit = dSurfaceX_m >= -.4 & dSurfaceX_m <= 1.2 & ...
    dSurfaceY_m >= -.1 & dSurfaceY_m <= .7;
strHits = QueryCameraSurface(objRequest, objRenderer, objCamera, ...
    [dColumns_px(:)'; dRows_px(:)']);

% Compare all three products to off-centre support, not merely to each other.
verifyEqual(testCase, ui8RawImage > 0, bExpectedHit);
verifyEqual(testCase, strLabels.status, uint32(bExpectedHit)*uint32(2));
verifyEqual(testCase, reshape(strHits.status == uint32(1), 64, 96), bExpectedHit);
verifyEqual(testCase, strLabels.class_id(bExpectedHit), ...
    repmat(bitshift(uint64(1), 53)+uint64(17), nnz(bExpectedHit), 1));
end

function testEmptyAndUnlabelledQueriesDoNotInventClasses(testCase)
% An empty query returns empty columns; an unlabelled mesh reports class 0 with no label validity.
RequireSpectraSurfaceQueries_(testCase);
[objRenderer, objCamera, objRequest, objCleanup] = BuildScene_(1000, false); %#ok<ASGLU>

strEmpty = QueryCameraSurface(objRequest, objRenderer, objCamera, zeros(2, 0));
verifySize(testCase, strEmpty.position_m, [0, 3]);
verifySize(testCase, strEmpty.class_id, [0, 1]);

strHits = QueryCameraSurface(objRequest, objRenderer, objCamera, [48.5; 32.5]);
verifyEqual(testCase, strHits.status, uint32(1));
verifyTrue(testCase, strHits.triangle_valid);
verifyEqual(testCase, bitand(strHits.field_validity, uint32(6)), uint32(0));
verifyEqual(testCase, strHits.class_id, uint64(0));
verifyEqual(testCase, strHits.position_m, [0, 0, 0], 'AbsTol', 2e-6);
end

function testUnsupportedProviderFailsBeforeQuery(testCase)
% A provider without the pixel-query interface fails before any renderer call.
objRequest = SImageRequest(uint32(1), int32(0), 0, [0; 0; -100], ...
    [0; 0; -3], eye(3), zeros(3, 1), eye(3));
objCamera = CCameraIntrinsics([45, 60], [48.5, 32.5], [64, 96]);
verifyError(testCase, @() QueryCameraSurface(objRequest, struct(), objCamera, [48.5; 32.5]), ...
    'QueryCameraSurface:UnsupportedRenderer');
end

function testMisregisteredCalibrationFailsAtRealRendererBoundary(testCase)
% The renderer rejects a principal point or focal length that differs from its viewport.
RequireSpectraSurfaceQueries_(testCase);
[objRenderer, ~, objRequest, objCleanup] = BuildScene_(1, true); %#ok<ASGLU>
cellCameras = {CCameraIntrinsics([45, 60], [48, 32], [64, 96]), ...
    CCameraIntrinsics([40, 60], [48.5, 32.5], [64, 96])};
cellReasons = {'centred', 'focal'};

for ui32CaseIdx = uint32(1):uint32(numel(cellCameras))
    bRejected = false;
    try
        QueryCameraSurface(objRequest, objRenderer, cellCameras{ui32CaseIdx}, [48.5; 32.5]);
    catch objError
        bRejected = contains(objError.message, cellReasons{ui32CaseIdx});
    end
    verifyTrue(testCase, bRejected, 'Reject calibration that is not represented by the rendered image.');
end
end

function RequireSpectraSurfaceQueries_(testCase)
% Skip the test unless a Spectra-RT wrapper with surface queries and scene files is on the path.
objClass = meta.class.fromName('spectra_rt.CSpectralRayTracerWrapper');
bAvailable = ~isempty(objClass);
if bAvailable
    bAvailable = all(ismember({'queryPixels', 'loadSceneFromConfig', 'renderFrameWithLabels'}, ...
        {objClass.MethodList.Name}));
end

assumeTrue(testCase, bAvailable, ['Needs a Spectra-RT build whose MATLAB wrapper has queryPixels, ', ...
    'loadSceneFromConfig and renderFrameWithLabels. Set SIM_GEARS_SPECTRA_RT_DIR to that build ', ...
    'before SetupSimGears.']);
end

function [objRenderer, objCamera, objRequest, objCleanup] = BuildScene_(dUnit_m, bLabels, bAsymmetric)
%% SIGNATURE
% [objRenderer, objCamera, objRequest, objCleanup] = BuildScene_(dUnit_m, bLabels, bAsymmetric)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build a 2-by-1.2 m plane (or an off-centre 1.6-by-0.8 m plane) 3 m in front of a 96-by-64
% camera, in a scene authored in metres and prepared in dUnit_m. With bLabels, load it through
% a labelled scene file; otherwise load a legacy OBJ with explicit units and no labels.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dUnit_m      Mesh length unit in metres (1 or 1000).
% bLabels      Load the labelled scene file instead of the legacy OBJ.
% bAsymmetric  Use the off-centre plane.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objRenderer  Built Spectra wrapper.
% objCamera    Matching CCameraIntrinsics.
% objRequest   Acquisition in a rotated, translated world frame, in dUnit_m.
% objCleanup   Releases the renderer and deletes the fixture folder.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dUnit_m (1,1) double
    bLabels (1,1) logical
    bAsymmetric (1,1) logical = false
end

% Write the plane mesh into a temporary folder
charFixtureDir = tempname;
mkdir(charFixtureDir);
charMeshPath = fullfile(charFixtureDir, 'plane.obj');
WriteText_(charMeshPath, sprintf(['o surface\nv -1 -.6 0\nv 1 -.6 0\n' ...
    'v 1 .6 0\nv -1 .6 0\nf 1 3 2\nf 1 4 3\n']));
if bAsymmetric
    WriteText_(charMeshPath, sprintf(['o surface\nv -.4 -.1 0\nv 1.2 -.1 0\n' ...
        'v 1.2 .7 0\nv -.4 .7 0\nf 1 3 2\nf 1 4 3\n']));
end

objRenderer = spectra_rt.CSpectralRayTracerWrapper(true);
objCleanup = onCleanup(@() ReleaseFixture_(objRenderer, charFixtureDir));

% Load the labelled scene, or the legacy OBJ path with explicit units
if bLabels
    charUnit = 'm';
    if dUnit_m == 1000
        charUnit = 'km';
    end

    % Describe one labelled asset and one placed instance; IDs above 2^53 need uint64
    strComponent = struct('object', 'surface', ...
        'component_id', uint64(0), ...
        'class_id', bitshift(uint64(1), 53) + uint64(17));

    strPreparation = struct('input_unit', 'm', ...
        'output_unit', charUnit, ...
        'frame', 'TARGET_BODY', ...
        'identity', struct('asset_id', 'analytic-plane', 'geometry_revision', 'v1'), ...
        'components', {{strComponent}});

    strAsset = struct('key', 'body', ...
        'mesh_model_path', 'plane.obj', ...
        'preparation', strPreparation);

    strInstance = struct('key', 'placed-body', ...
        'asset_key', 'body', ...
        'body_instance_id', bitshift(uint64(1), 53) + uint64(1), ...
        'position_w', [0, 0, 0], ...
        'quaternion_wxyz', [1, 0, 0, 0]);

    charScenePath = fullfile(charFixtureDir, 'plane.yml');
    yaml.dumpFile(charScenePath, struct('assets', {{strAsset}}, 'instances', {{strInstance}}));
    objRenderer.loadSceneFromConfig(charScenePath);
else
    % Exercise the legacy-OBJ path with explicit units and no labels: overwrite the plane with the
    % same plane written in kilometres (callers pass dUnit_m = 1000 here).
    WriteText_(charMeshPath, sprintf(['o surface\nv -.001 -.0006 0\nv .001 -.0006 0\n' ...
        'v .001 .0006 0\nv -.001 .0006 0\nf 1 3 2\nf 1 4 3\n']));
    objRenderer.loadMeshModelWithUnits(charMeshPath, dUnit_m, false);
end

% Configure a 96-by-64 pinhole camera 3 m from the plane
objCamera = CCameraIntrinsics([45, 60], [48.5, 32.5], [64, 96]);
objRenderer.setCameraFromLookAt([0; 0; -3]/dUnit_m, zeros(3, 1), [0; 1; 0], ...
    [96; 64], [2*atan(96/90); 2*atan(64/120)]);
objRenderer.setOutputModeString('RGBA8_ONLY');
objRenderer.setPixelSamplingModeString('FIXED_MSAA');
objRenderer.setNumSampleRaysPerPixel(uint32(1));
objRenderer.setNumDirectLightSamplesPerLight(uint32(1));
objRenderer.buildGeometryFromLoadedModel();
objRenderer.buildPipelineAndSBT();

% Express the same acquisition in a rotated, translated world frame
dDCM_WfromTB = [0, -1, 0; 1, 0, 0; 0, 0, 1];
dTargetPosition_W_m = [10; 20; 30];
objRequest = SImageRequest(uint32(17), int32(4), 1.75, ...
    (dTargetPosition_W_m + dDCM_WfromTB*[0; 0; -100])/dUnit_m, ...
    (dTargetPosition_W_m + dDCM_WfromTB*[0; 0; -3])/dUnit_m, ...
    dDCM_WfromTB, dTargetPosition_W_m/dUnit_m, dDCM_WfromTB, 'dLengthUnit_m', dUnit_m);
end

function ReleaseFixture_(objRenderer, charFixtureDir)
% Release the renderer, then delete the fixture folder.
arguments (Input)
    objRenderer (1,1) spectra_rt.CSpectralRayTracerWrapper
    charFixtureDir (1,:) char
end

objRenderer.cleanup();
rmdir(charFixtureDir, 's');
end

function WriteText_(charPath, charText)
% Write text and close the file before the renderer loads it.
arguments (Input)
    charPath (1,:) char
    charText (1,:) char
end

dFileID = fopen(charPath, 'w');
assert(dFileID >= 0, 'testCameraSurfaceQueries:FixtureWrite', 'Cannot create analytic fixture.');
objCleanup = onCleanup(@() fclose(dFileID));
fprintf(dFileID, '%s', charText);
end
