function [objShapeModel, objFig] = DemoApophisShapeModel(charDataRootPath)
%% SIGNATURE
% [objShapeModel, objFig] = DemoApophisShapeModel(charDataRootPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Load the registry-backed Apophis shape model and display it in the navigation
% frame. This manual integration example requires the Apophis shape asset
% declared by data/scenarios/Apophis/manifest.json.
%
% Example:
%   [objShapeModel, objFig] = DemoApophisShapeModel('/path/to/simgears/data');
%
% Expected output:
%   A CShapeModel instance and an interactive figure containing the Apophis
%   triangular mesh, camera direction, and Sun direction.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charDataRootPath    SimulationGears data root containing the scenario assets.
%                     An empty value uses SIMGEARS_DATA_ROOT or the repository data directory.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objShapeModel    Registry-backed Apophis CShapeModel instance.
% objFig           Figure containing the shape-model visualization.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-07-2026    Pietro Califano, Codex     Replace obsolete setup and hard-coded SPICE paths with registry routing.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% DefineShapeModel
% Visualize3dShapeModelWithPC
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    charDataRootPath (1, :) char = ''
end

arguments (Output)
    objShapeModel (1, 1) CShapeModel
    objFig (1, 1) matlab.ui.Figure
end

charExampleDirectory = fileparts(mfilename('fullpath'));
charRepositoryRoot = fileparts(fileparts(fileparts(charExampleDirectory)));
run(fullfile(charRepositoryRoot, 'matlab', 'SetupSimGears.m'));

% Resolve the current registry-backed shape source without loading obsolete
% trajectory enums, navigation configuration scripts, or mission kernels.
[objShapeModel, ~, ~] = DefineShapeModel( ...
    "Apophis", ...
    charDataRootPath, ...
    bInitSphericalHarmonicsGravityData=false);

dCameraPosition_NavFrame = 1.0e3 .* [1.0; 0.1; 0.0];
dSunPosition_NavFrame = 1.0e9 .* [1.0; 0.0; 0.0];
dBodyDCM_NavFrameFromOF = eye(3);
[objFig, ~] = Visualize3dShapeModelWithPC( ...
    objShapeModel.getShapeStruct(), ...
    dCameraPosition_NavFrame, ...
    dSunPosition_NavFrame, ...
    dBodyDCM_NavFrameFromOF, ...
    'bEnableLegend', false);

end
