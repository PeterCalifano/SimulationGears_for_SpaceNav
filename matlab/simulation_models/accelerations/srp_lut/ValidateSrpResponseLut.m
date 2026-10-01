function ValidateSrpResponseLut(strResponseLut)
%% SIGNATURE
% ValidateSrpResponseLut(strResponseLut)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate a regular spacecraft SRP table before repeated interpolation. Require
% the complete sphere, finite scalar/vector values, SI response units, matching
% periodic seam/pole values, and nodal transverse orthogonality. Supply fixed
% geometry/optics provenance in the caller and apply pressure, mass, attitude
% rotation and external eclipse independently.
% Example: ValidateSrpResponseLut(strResponseLut);
% Output: Accept a consistent table or raise an identified contract error.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strResponseLut   Scalar or transverse numeric schema packed by PackSrpResponseLut, up to
%                  361 x 181 nodes and uint32 active counts. Axes [deg],
%                  Cr [-], force/pressure [m^2], reference area [m^2].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None. Reject malformed axes, response values, units or boundary conventions.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 28-09-2026  Pietro Califano, Codex gpt-6  Formalize the optional scalar/vector LUT.
% 28-09-2026  Pietro Califano, Codex gpt-6  Check the schema without allocating a prototype.
% 29-09-2026  Pietro Califano, Codex gpt-6  Validate compact capacities without changing fields.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None; validate host payloads before naming their generated SSrpResponseLut type.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strResponseLut (1, 1) struct
end

% Require explicit dimensions and units before reading any grid coordinates.
cellRequiredFieldNames = {'ui32AzimuthCount';'ui32ElevationCount';'dAzimuth';'dElevation'; ...
    'dEffectiveCr';'dReferenceArea_m2'};
bIncludeTransverse = isfield(strResponseLut, 'dTransverseForcePerPressure');
if bIncludeTransverse
    cellRequiredFieldNames{end + 1} = 'dTransverseForcePerPressure';
end
assert(all(isfield(strResponseLut, cellRequiredFieldNames)), 'ValidateSrpResponseLut:MissingField', ...
    'Supply every fixed-schema scalar/vector field.');
assert(isequal(sort(fieldnames(strResponseLut)), sort(cellRequiredFieldNames)), ...
    'ValidateSrpResponseLut:InvalidSchema', 'Keep host metadata outside the numeric response schema.');

% Check storage dimensions before constructing expected field shapes.
dAzimuthCapacity = size(strResponseLut.dAzimuth, 2);
dElevationCapacity = size(strResponseLut.dElevation, 2);
assert(dAzimuthCapacity >= 3 && dAzimuthCapacity <= 361 && ...
    dElevationCapacity >= 3 && dElevationCapacity <= 181, ...
    'ValidateSrpResponseLut:CapacityExceeded', 'Keep fixed capacities within 361 by 181 nodes.');
cellExpectedFieldShapes = {[1, 1], [1, 1], [1, dAzimuthCapacity], [1, dElevationCapacity], ...
    [dElevationCapacity, dAzimuthCapacity], [1, 1]};
if bIncludeTransverse
    cellExpectedFieldShapes{end + 1} = [3, dElevationCapacity, dAzimuthCapacity];
end
for ui32Field = uint32(1):uint32(numel(cellRequiredFieldNames))
    charFieldName = cellRequiredFieldNames{ui32Field};
    charExpectedType = 'double';
    if ui32Field <= 2
        charExpectedType = 'uint32';
    end
    assert(isa(strResponseLut.(charFieldName), charExpectedType) && ...
        isreal(strResponseLut.(charFieldName)) && ~issparse(strResponseLut.(charFieldName)) && ...
        isequal(size(strResponseLut.(charFieldName)), cellExpectedFieldShapes{ui32Field}), ...
        'ValidateSrpResponseLut:InvalidSchema', 'Require dense real numeric fields with fixed types and shapes.');
end

% Validate active counts before slicing populated interpolation axes.
ui32AzimuthCount = strResponseLut.ui32AzimuthCount;
ui32ElevationCount = strResponseLut.ui32ElevationCount;
assert(ui32AzimuthCount >= 3 && ui32AzimuthCount <= dAzimuthCapacity && ...
    ui32ElevationCount >= 3 && ui32ElevationCount <= dElevationCapacity, ...
    'ValidateSrpResponseLut:CapacityExceeded', 'Require populated counts within fixed capacity.');

% Require uniform, ordered axes covering the complete sphere.
dAzimuth = strResponseLut.dAzimuth(1:ui32AzimuthCount);
dElevation = strResponseLut.dElevation(1:ui32ElevationCount);
assert(isrow(dAzimuth) && isrow(dElevation) && numel(dAzimuth) >= 3 && ...
    numel(dElevation) >= 3 && all(isfinite([dAzimuth, dElevation])) && ...
    dAzimuth(1) == -180 && dAzimuth(end) == 180 && ...
    dElevation(1) == -90 && dElevation(end) == 90 && ...
    all(diff(dAzimuth) > 0) && all(diff(dElevation) > 0), ...
    'ValidateSrpResponseLut:InvalidAxes', 'Require ordered complete-sphere azimuth/elevation axes.');
assert(max(abs(diff(dAzimuth)-diff(dAzimuth(1:2)))) < 1e-10 && ...
    max(abs(diff(dElevation)-diff(dElevation(1:2)))) < 1e-10, ...
    'ValidateSrpResponseLut:InvalidAxes', 'Require uniform spacing on each interpolation axis.');
assert(isfinite(strResponseLut.dReferenceArea_m2) && strResponseLut.dReferenceArea_m2 > 0, ...
    'ValidateSrpResponseLut:InvalidUnits', 'Require positive reference area in m^2.');

% Validate scalar values and boundaries independently of transverse storage.
dEffectiveCrGrid = strResponseLut.dEffectiveCr(1:ui32ElevationCount, 1:ui32AzimuthCount);
assert(all(isfinite(strResponseLut.dEffectiveCr), 'all') && all(dEffectiveCrGrid >= 0, 'all'), ...
    'ValidateSrpResponseLut:InvalidValues', 'Require finite nonnegative scalar response values.');
assert(isequal(dEffectiveCrGrid(:, 1), dEffectiveCrGrid(:, end)) && ...
    all(dEffectiveCrGrid(1, :) == dEffectiveCrGrid(1, 1)) && ...
    all(dEffectiveCrGrid(end, :) == dEffectiveCrGrid(end, 1)), ...
    'ValidateSrpResponseLut:InvalidBoundary', 'Require identical scalar seam and pole values.');
assert(all(strResponseLut.dAzimuth(ui32AzimuthCount+1:end) == 0) && ...
    all(strResponseLut.dElevation(ui32ElevationCount+1:end) == 0) && ...
    all(strResponseLut.dEffectiveCr(ui32ElevationCount+1:end, :) == 0, 'all') && ...
    all(strResponseLut.dEffectiveCr(:, ui32AzimuthCount+1:end) == 0, 'all'), ...
    'ValidateSrpResponseLut:InvalidPadding', 'Keep every inactive scalar table value zero.');

if bIncludeTransverse
    % Require each stored vector to lie in its node's transverse plane.
    dTransverseGrid = strResponseLut.dTransverseForcePerPressure(:, 1:ui32ElevationCount, 1:ui32AzimuthCount);
    assert(all(isfinite(strResponseLut.dTransverseForcePerPressure), 'all'), ...
        'ValidateSrpResponseLut:InvalidValues', 'Require finite transverse samples.');
    [dAzimuthGrid, dElevationGrid] = meshgrid(dAzimuth, dElevation);
    dParallelComponent = squeeze(dTransverseGrid(1, :, :)).*cosd(dElevationGrid).*cosd(dAzimuthGrid) + ...
        squeeze(dTransverseGrid(2, :, :)).*cosd(dElevationGrid).*sind(dAzimuthGrid) + ...
        squeeze(dTransverseGrid(3, :, :)).*sind(dElevationGrid);
    assert(max(abs(dParallelComponent), [], 'all') <= ...
        1e-10 * max(1, max(abs(dTransverseGrid), [], 'all')), ...
        'ValidateSrpResponseLut:InconsistentProjection', 'Require nodal transverse orthogonality.');

    % Preserve exact vector boundaries and canonical inactive storage.
    assert(isequal(dTransverseGrid(:, :, 1), dTransverseGrid(:, :, end)) && ...
        isequal(dTransverseGrid(:, 1, :), repmat(dTransverseGrid(:, 1, 1), 1, 1, numel(dAzimuth))) && ...
        isequal(dTransverseGrid(:, end, :), repmat(dTransverseGrid(:, end, 1), 1, 1, numel(dAzimuth))), ...
        'ValidateSrpResponseLut:InvalidBoundary', 'Require identical transverse seam and pole values.');
    assert(all(strResponseLut.dTransverseForcePerPressure(:, ui32ElevationCount+1:end, :) == 0, 'all') && ...
        all(strResponseLut.dTransverseForcePerPressure(:, :, ui32AzimuthCount+1:end) == 0, 'all'), ...
        'ValidateSrpResponseLut:InvalidPadding', 'Keep every inactive transverse sample zero.');
end

coder.cstructname(strResponseLut, 'SSrpResponseLut');
end
