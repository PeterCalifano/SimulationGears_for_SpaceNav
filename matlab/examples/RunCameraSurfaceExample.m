function [varImage, strHits, strLabels] = RunCameraSurfaceExample()
%% SIGNATURE
% [varImage, strHits, strLabels] = RunCameraSurfaceExample()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Render a small labelled plane and query two image coordinates through the selected Spectra MEX.
% Build the fixture locally; no shared asset folder is required. Prepare the metre-authored OBJ
% in kilometres, then return query coordinates in metres. Use modern MATLAB K and [H,W] sizing.
% Expected output: a 24-by-32 image, class ID 42 for both hits, and positions [0,0,0;0.6,0.25,0] m.
% The example deletes its temporary input files and releases the renderer before returning.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None. Add SimulationGears, Spectra MATLAB helpers and its matching wrapper/MEX build first.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% varImage   Grayscale image before any optional image-error pipeline.
% strHits    Sparse records for the two query coordinates, with exact IDs and validity.
% strLabels  Same-render dense labels; categorical IDs are not display intensities.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-10-2026  Pietro Califano, Codex GPT-6  Demonstrate acquisition-bound surface queries and dense labels.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; write the scene file with yaml.dumpFile.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SImageRequest, CCameraIntrinsics, RenderCameraFrame, QueryCameraSurface, Spectra MEX, yaml.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
end
arguments (Output)
    varImage (:,:) {mustBeNumeric}
    strHits (1,1) struct
    strLabels (1,1) struct
end

% Write a labelled square plane and its scene description to a temporary folder
charFixtureDir = tempname;
mkdir(charFixtureDir);
objFileCleanup = onCleanup(@() rmdir(charFixtureDir, 's'));

WriteText_(fullfile(charFixtureDir, 'plane.obj'), sprintf( ...
    'o surface\nv -1 -1 0\nv 1 -1 0\nv 1 1 0\nv -1 1 0\nf 1 3 2\nf 1 4 3\n'));

% Describe the plane as one labelled asset authored in metres and prepared in kilometres
strComponent = struct('object', 'surface', ...
    'component_id', uint64(0), ...
    'class_id', uint64(42));

strPreparation = struct('input_unit', 'm', ...
    'output_unit', 'km', ...
    'frame', 'TARGET_BODY', ...
    'identity', struct('asset_id', 'example-plane', 'geometry_revision', 'v1'), ...
    'components', {{strComponent}});

strAsset = struct('key', 'plane', ...
    'mesh_model_path', 'plane.obj', ...
    'preparation', strPreparation);

charScenePath = fullfile(charFixtureDir, 'scene.yml');
yaml.dumpFile(charScenePath, struct('assets', {{strAsset}}));

% Load the scene and configure a 24-by-32 camera looking at the plane
objRenderer = spectra_rt.CSpectralRayTracerWrapper(true);
objRendererCleanup = onCleanup(@() objRenderer.cleanup());
objRenderer.loadSceneFromConfig(charScenePath);

objCamera = CCameraIntrinsics([20, 24], [16.5, 12.5], [24, 32]);
objRenderer.setCameraFromLookAt([0; 0; -.003], zeros(3, 1), [0; 1; 0], ...
    flip(objCamera.ImageSize(:)), flip(objCamera.dFovHW));
objRenderer.setOutputModeString('RGBA8_ONLY');
objRenderer.setNumSampleRaysPerPixel(uint32(1));
objRenderer.setNumDirectLightSamplesPerLight(uint32(1));
objRenderer.buildGeometryFromLoadedModel();
objRenderer.buildPipelineAndSBT();

% Use the same acquisition for rendering and queries; positions here are kilometres.
objRequest = SImageRequest(uint32(1), int32(0), 0, [0; 0; -.1], ...
    [0; 0; -.003], eye(3), zeros(3, 1), eye(3), 'dLengthUnit_m', 1000);

[varImage, ~, ~, strLabels] = RenderCameraFrame(objRequest, objRenderer, ...
    'bCaptureSurfaceLabels', true);
strHits = QueryCameraSurface(objRequest, objRenderer, objCamera, [16.5, 20.5; 12.5, 14.5]);

fprintf('Surface example: %d x %d pixels, %d valid triangle hits, class IDs %u and %u\n', ...
    size(varImage, 1), size(varImage, 2), nnz(strHits.triangle_valid), ...
    strHits.class_id(1), strHits.class_id(2));
end

function WriteText_(charPath, charText)
%% SIGNATURE
% WriteText_(charPath, charText)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Write charText to charPath and close the file before the native renderer loads it.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charPath  Output file path.
% charText  Text written as is.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charPath (1,:) char
    charText (1,:) char
end

dFileID = fopen(charPath, 'w');
assert(dFileID >= 0, 'RunCameraSurfaceExample:FixtureWrite', 'Cannot create example fixture.');
objCleanup = onCleanup(@() fclose(dFileID));
fprintf(dFileID, '%s', charText);
end
