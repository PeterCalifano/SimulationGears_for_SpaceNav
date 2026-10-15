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
% Inspect the fixed numeric schema without cell arrays, including when guidance
% validates a runtime table update before atomic publication.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strResponseLut   Scalar or transverse numeric schema packed by PackSrpResponseLut, up to
%                  721 x 361 nodes and uint32 active counts. Axes [deg],
%                  Cr [-], force/pressure [m^2], reference area [m^2], optional torque [m^3].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None. Reject malformed axes, response values, units or boundary conventions.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-09-2026  Pietro Califano, Codex GPT-6    Formalize the optional scalar/vector LUT.
% 28-09-2026  Pietro Califano, Codex GPT-6    Check the schema without allocating a prototype.
% 29-09-2026  Pietro Califano, Codex GPT-6    Validate compact capacities without changing fields.
% 01-10-2026  Pietro Califano, Codex GPT-6    Correct nodal transverse samples and constant inclusion.
% 01-10-2026  Pietro Califano, Codex GPT-6    Clarify variable roles and separate computation steps.
% 06-10-2026  Pietro Califano, Codex GPT-6    Validate half-degree capacities and optional torque boundaries.
% 14-10-2026  Pietro Califano, Codex GPT-6.1    Validate runtime schemas without cell reflection.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None; validate host payloads before naming their generated SSrpResponseLut type.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strResponseLut (1, 1) struct
end

% Require explicit dimensions and units before reading any grid coordinates.
assert(isfield(strResponseLut, 'ui32AzimuthCount') && ...
    isfield(strResponseLut, 'ui32ElevationCount') && ...
    isfield(strResponseLut, 'dAzimuth') && isfield(strResponseLut, 'dElevation') && ...
    isfield(strResponseLut, 'dEffectiveCr') && isfield(strResponseLut, 'dReferenceArea_m2'), ...
    'ValidateSrpResponseLut:MissingField', 'Supply every fixed-schema scalar/vector field.');

% Build the permitted field set from existing values. MATLAB shares these arrays;
% the comparison rejects extra fields without allocating a second response grid.
strNumericSchema = struct( ...
    'ui32AzimuthCount', strResponseLut.ui32AzimuthCount, ...
    'ui32ElevationCount', strResponseLut.ui32ElevationCount, ...
    'dAzimuth', strResponseLut.dAzimuth, ...
    'dElevation', strResponseLut.dElevation, ...
    'dEffectiveCr', strResponseLut.dEffectiveCr, ...
    'dReferenceArea_m2', strResponseLut.dReferenceArea_m2);
bIncludeTransverse = isfield(strResponseLut, 'dTransverseForcePerPressure');
if bIncludeTransverse
    strNumericSchema.dTransverseForcePerPressure = strResponseLut.dTransverseForcePerPressure;
end
bIncludeTorque = isfield(strResponseLut,'dTorquePerPressure');
if bIncludeTorque
    strNumericSchema.dTorquePerPressure = strResponseLut.dTorquePerPressure;
end
assert(isequaln(strResponseLut, strNumericSchema), ...
    'ValidateSrpResponseLut:InvalidSchema', 'Keep host metadata outside the numeric response schema.');

% Check storage dimensions before constructing expected field shapes.
dAzimuthCapacity = size(strResponseLut.dAzimuth, 2);
dElevationCapacity = size(strResponseLut.dElevation, 2);
assert(dAzimuthCapacity >= 3 && dAzimuthCapacity <= 721 && ...
    dElevationCapacity >= 3 && dElevationCapacity <= 361, ...
    'ValidateSrpResponseLut:CapacityExceeded', 'Keep fixed capacities within 721 by 361 nodes.');
ValidateNumericField_(strResponseLut.ui32AzimuthCount, 'uint32', [1, 1]);
ValidateNumericField_(strResponseLut.ui32ElevationCount, 'uint32', [1, 1]);
ValidateNumericField_(strResponseLut.dAzimuth, 'double', [1, dAzimuthCapacity]);
ValidateNumericField_(strResponseLut.dElevation, 'double', [1, dElevationCapacity]);
ValidateNumericField_(strResponseLut.dEffectiveCr, 'double', [dElevationCapacity, dAzimuthCapacity]);
ValidateNumericField_(strResponseLut.dReferenceArea_m2, 'double', [1, 1]);
if bIncludeTransverse
    ValidateNumericField_(strResponseLut.dTransverseForcePerPressure, ...
        'double', [3, dElevationCapacity, dAzimuthCapacity]);
end
if bIncludeTorque
    ValidateNumericField_(strResponseLut.dTorquePerPressure, ...
        'double', [3, dElevationCapacity, dAzimuthCapacity]);
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

if bIncludeTorque
    dTorqueGrid = strResponseLut.dTorquePerPressure(:,1:ui32ElevationCount,1:ui32AzimuthCount);
    assert(all(isfinite(strResponseLut.dTorquePerPressure),'all'), ...
        'ValidateSrpResponseLut:InvalidValues','Require finite body-origin torque samples.');
    assert(isequal(dTorqueGrid(:,:,1),dTorqueGrid(:,:,end)) && ...
        isequal(dTorqueGrid(:,1,:),repmat(dTorqueGrid(:,1,1),1,1,numel(dAzimuth))) && ...
        isequal(dTorqueGrid(:,end,:),repmat(dTorqueGrid(:,end,1),1,1,numel(dAzimuth))), ...
        'ValidateSrpResponseLut:InvalidBoundary','Require identical torque seam and pole values.');
    assert(all(strResponseLut.dTorquePerPressure(:,ui32ElevationCount+1:end,:)==0,'all') && ...
        all(strResponseLut.dTorquePerPressure(:,:,ui32AzimuthCount+1:end)==0,'all'), ...
        'ValidateSrpResponseLut:InvalidPadding','Keep inactive torque values zero.');
end
coder.cstructname(strResponseLut, 'SSrpResponseLut');
end

function ValidateNumericField_(dValue, charExpectedType, dExpectedShape)
%% SIGNATURE
% ValidateNumericField_(dValue, charExpectedType, dExpectedShape)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Enforce one fixed numeric field's storage type and shape.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dValue             Table field before validation.
% charExpectedType    Required MATLAB numeric class.
% dExpectedShape      Required dimensions, including scalar and vector fields.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; reject sparse, complex, mistyped or incorrectly shaped fields.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 14-10-2026  Pietro Califano, Codex GPT-6.1    Share explicit field checks without cell inspection.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dValue
    charExpectedType (1,:) char
    dExpectedShape (1,:) double
end

arguments (Output)
end

assert(isa(dValue, charExpectedType) && isreal(dValue) && ~issparse(dValue) && ...
    isequal(size(dValue), dExpectedShape), ...
    'ValidateSrpResponseLut:InvalidSchema', ...
    'Require dense real numeric fields with fixed types and shapes.');

end
