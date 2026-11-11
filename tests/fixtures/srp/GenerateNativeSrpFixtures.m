function GenerateNativeSrpFixtures(charReferenceRoot, charOutputDirectory)
%% SIGNATURE
% GenerateNativeSrpFixtures(charReferenceRoot, charOutputDirectory)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate a small independent panel-law oracle from the established MATLAB
% implementation. Reference sources and hashes accompany the saved fixtures.
% This is an explicit preparation tool, never part of automatic native tests.
% Example: GenerateNativeSrpFixtures('/path/to/SimulationGears','/tmp/srp-fixtures');
% Output: matlab_panel.csv and matlab_response.json with deterministic values.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charReferenceRoot Repository containing the established ComputePanelSrpResponse.
% charOutputDirectory Existing destination for reference data.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2026  Pietro Califano  Add independent native panel/LUT reference generation.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; add the srp/panels and srp/lut folders.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePanelSrpResponse, ExportSrpResponseLutJson, PackSrpResponseLut, EvalJac_SrpResponseLut.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    charReferenceRoot (1,:) char
    charOutputDirectory (1,:) char
end

% Add the reference panel and LUT folders; restore the caller path on exit
charOriginalPath = path;
objCleanup = onCleanup(@() path(charOriginalPath));
charSrpRoot = fullfile(charReferenceRoot, 'matlab', 'simulation_models', 'accelerations', 'srp');
addpath(fullfile(charSrpRoot, 'panels'));
addpath(fullfile(charSrpRoot, 'lut'));

% Evaluate force, Jacobian and torque of two frozen-visibility panels at a few queries
strPanel = struct('dSCquadsArea', [0.5; 0.5], ...
                  'dDiffSpecQuadsCoeffs', [0.2, 0.6; 0.1, 0.3], ...
                  'dQuadsNormals_SCB', [0, 1; 0, 0; 1, 0], ...
                  'dQuadsPressCentre_SCB', [1/3, 0; 1/3, 1/3; 0, 1/3]);
dQueries = [0.3, -0.5, 1.7; 1.1, 0.8, -0.2; -0.3, 0.7, 2; 1, 1, 1; -1, -1, -1].';
dVisibility = repmat([0.375; 0.625], 1, size(dQueries, 2));
[dForce, dJacobian, dTorque] = ComputePanelSrpResponse(dQueries, strPanel, false, dVisibility);
dOutput = [dQueries; dForce; reshape(dJacobian, 9, []); dTorque].';
writematrix(dOutput, fullfile(charOutputDirectory, 'matlab_panel.csv'));

% Generate direct force responses at every node, then copy identical poles/seams.
dAzimuth = -180:30:180;
dElevation = -90:30:90;
[dAzimuthGrid, dElevationGrid] = meshgrid(dAzimuth, dElevation);
dDirections = [cosd(dElevationGrid(:)).'.*cosd(dAzimuthGrid(:)).'; ...
    cosd(dElevationGrid(:)).'.*sind(dAzimuthGrid(:)).'; sind(dElevationGrid(:)).'];
dForce = ComputePanelSrpResponse(dDirections, strPanel, false);
dParallel = sum(dDirections.*dForce, 1);
dCr = reshape(-dParallel/0.73, numel(dElevation), numel(dAzimuth));
dTransverse = reshape(dForce-dDirections.*dParallel, 3, numel(dElevation), numel(dAzimuth));
dCr(:, end) = dCr(:, 1);
dCr(1, :) = dCr(1, 1);
dCr(end, :) = dCr(end, 1);
dTransverse(:, :, end) = dTransverse(:, :, 1);
dTransverse(:, 1, :) = repmat(dTransverse(:, 1, 1), 1, 1, numel(dAzimuth));
dTransverse(:, end, :) = repmat(dTransverse(:, end, 1), 1, 1, numel(dAzimuth));
strTable = struct('dAzimuth', dAzimuth, ...
                  'dElevation', dElevation, ...
                  'dEffectiveCr', dCr, ...
                  'dReferenceArea_m2', 0.73, ...
                  'dTransverseForcePerPressure', dTransverse);
ExportSrpResponseLutJson(strTable, fullfile(charOutputDirectory, 'matlab_response.json'));

% Exercise the existing spherical kernel, including poles, seam sides and extreme query scales.
strPayload = PackSrpResponseLut(strTable, bIncludeTransverse=true, ...
    ui32Capacity=uint32([numel(dAzimuth), numel(dElevation)]));
dQueries = [1, 0.2, 0.3; -1, 1e-8, 0.3; -1, -1e-8, 0.3; 0, 0, 1; 0, 0, -1; ...
    1e-9, 0, 1; 3e-9, 0, 1; 1e200, 2e199, 3e199; 1e-200, 2e-201, 3e-201].';
dOutput = zeros(size(dQueries, 2), 17);
for ui32QueryIdx = uint32(1):uint32(size(dQueries, 2))
    [dJacobian, ~, ~, dForce, dCr, ~, bRegular] = ...
        EvalJac_SrpResponseLut(dQueries(:, ui32QueryIdx), strPayload, true);
    dOutput(ui32QueryIdx, :) = [dQueries(:, ui32QueryIdx).', dForce.', dJacobian(:).', dCr, double(bRegular)];
end
writematrix(dOutput, fullfile(charOutputDirectory, 'matlab_kernel.csv'));
end
