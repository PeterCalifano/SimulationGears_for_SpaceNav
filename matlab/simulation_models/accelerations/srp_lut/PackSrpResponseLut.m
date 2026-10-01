function strResponseLut = PackSrpResponseLut(strTable, kwargs)
%% SIGNATURE
% strResponseLut = PackSrpResponseLut(strTable)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Pack compact host table values into the fixed numeric SRP schema. Reject
% capacity overflow rather than truncating or resampling. Preserve host metadata
% outside the numeric payload and zero every inactive array element.
% Example: strResponseLut = PackSrpResponseLut(strSaved.strLut);
% Output: A validated fixed-shape table usable by source, MEX and C++ consumers.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strTable                 Compact dAzimuth/dElevation [deg], dEffectiveCr [-],
%                          dReferenceArea_m2 and, when selected,
%                          dTransverseForcePerPressure (3, Elevation, Azimuth) [m^2].
% kwargs.bIncludeTransverse Include transverse samples; default false.
% kwargs.ui32Capacity       Fixed [azimuth, elevation] capacity; default [361, 181].
%                          Use [73, 37] for a compact 5-degree codegen specialization.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strResponseLut        Six numeric fields and an optional transverse array,
%                       with fixed selected storage up to 361 x 181 nodes.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 28-09-2026  Pietro Califano, Codex gpt-6  Pack the host artifact into bounded storage.
% 28-09-2026  Pietro Califano, Codex gpt-6  Initialize the fixed payload directly.
% 29-09-2026  Pietro Califano, Codex gpt-6  Support fixed capacities matched to the selected grid.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ValidateSrpResponseLut.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strTable (1, 1) struct
    kwargs.bIncludeTransverse (1, 1) logical = false
    kwargs.ui32Capacity (1, 2) uint32 = uint32([361, 181])
end

arguments (Output)
    strResponseLut (1, 1) struct
end

% Check source fields and storage limits before allocating fixed numeric arrays.
assert(all(isfield(strTable, {'dAzimuth', 'dElevation', 'dEffectiveCr', ...
                            'dReferenceArea_m2'})), ...
    'PackSrpResponseLut:MissingField', 'Supply all compact response values.');
ui32Capacity = kwargs.ui32Capacity;
assert(all(ui32Capacity >= 3) && all(ui32Capacity <= uint32([361, 181])), ...
    'PackSrpResponseLut:CapacityExceeded', 'Keep fixed capacities within 361 by 181 nodes.');

% Keep active counts distinct from the selected storage capacities.
ui32AzimuthCount = uint32(numel(strTable.dAzimuth));
ui32ElevationCount = uint32(numel(strTable.dElevation));
assert(ui32AzimuthCount >= 3 && ui32AzimuthCount <= ui32Capacity(1) && ...
    ui32ElevationCount >= 3 && ui32ElevationCount <= ui32Capacity(2), ...
    'PackSrpResponseLut:CapacityExceeded', 'Require at most 361 azimuth and 181 elevation nodes.');
assert(isequal(size(strTable.dEffectiveCr), [double(ui32ElevationCount), double(ui32AzimuthCount)]), ...
    'PackSrpResponseLut:InvalidValues', 'Require compact values consistent with the axes.');

% Allocate the complete fixed payload here and leave inactive storage zero.
strResponseLut = struct('ui32AzimuthCount', ui32AzimuthCount, 'ui32ElevationCount', ui32ElevationCount, ...
    'dAzimuth', zeros(1, ui32Capacity(1)), 'dElevation', zeros(1, ui32Capacity(2)), ...
    'dEffectiveCr', zeros(ui32Capacity(2), ui32Capacity(1)), ...
    'dReferenceArea_m2', strTable.dReferenceArea_m2);

% Copy only populated nodes and retain canonical zero padding.
strResponseLut.dAzimuth(1:ui32AzimuthCount) = strTable.dAzimuth;
strResponseLut.dElevation(1:ui32ElevationCount) = strTable.dElevation;
strResponseLut.dEffectiveCr(1:ui32ElevationCount, 1:ui32AzimuthCount) = strTable.dEffectiveCr;
if kwargs.bIncludeTransverse
    % Allocate the vector payload only when the selected model uses it.
    assert(isequal(size(strTable.dTransverseForcePerPressure), ...
                   [3, double(ui32ElevationCount), double(ui32AzimuthCount)]), ...
        'PackSrpResponseLut:InvalidValues', 'Require transverse samples consistent with the axes.');
    strResponseLut.dTransverseForcePerPressure = zeros(3, ui32Capacity(2), ui32Capacity(1));
    strResponseLut.dTransverseForcePerPressure(:, 1:ui32ElevationCount, 1:ui32AzimuthCount) = ...
        strTable.dTransverseForcePerPressure;
end

ValidateSrpResponseLut(strResponseLut);
coder.cstructname(strResponseLut, 'SSrpResponseLut');
end
