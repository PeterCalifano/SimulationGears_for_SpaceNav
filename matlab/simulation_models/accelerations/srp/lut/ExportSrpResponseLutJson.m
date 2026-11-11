function ExportSrpResponseLutJson(strTable, charOutputPath)
%% SIGNATURE
% ExportSrpResponseLutJson(strTable, charOutputPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Export active force samples from a compact or fixed-capacity MATLAB payload
% to SimulationGears.SrpResponseLut.v1 JSON. Explicitly omit optional torque.
% Preserve channel/elevation/azimuth order and serialize SI response values.
% Example: ExportSrpResponseLutJson(strTable, 'response.json');
% Output: A native-readable response.json file containing the active table.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strTable       Compact table or fixed payload with active ui32*Count fields.
% charOutputPath Destination JSON file, replaced by this explicit operation.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2026  Pietro Califano  Add native portable force-table interchange.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; move to srp/lut.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ValidateSrpLutJsonArtifact_, jsonencode.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    strTable (1,1) struct
    charOutputPath (1,:) char
end

% Extract only active entries; fixed-capacity padding has no native meaning.
dAzimuthCount = numel(strTable.dAzimuth);
dElevationCount = numel(strTable.dElevation);
if isfield(strTable, 'ui32AzimuthCount')
    dAzimuthCount = double(strTable.ui32AzimuthCount);
    dElevationCount = double(strTable.ui32ElevationCount);
end
validateattributes(dAzimuthCount, {'double'}, {'scalar', 'integer', '>=', 3});
validateattributes(dElevationCount, {'double'}, {'scalar', 'integer', '>=', 3});

% Stack Cr and, when present, the transverse force in channel-fast order
dSamples = reshape(strTable.dEffectiveCr(1:dElevationCount, 1:dAzimuthCount), 1, []);
cellChannels = {'Cr'};
cellUnits = {'1'};
if isfield(strTable, 'dTransverseForcePerPressure')
    dSamples = [dSamples; reshape(strTable.dTransverseForcePerPressure( ...
        :, 1:dElevationCount, 1:dAzimuthCount), 3, [])];
    cellChannels = {'Cr', 'Tx', 'Ty', 'Tz'};
    cellUnits = {'1', 'm^2', 'm^2', 'm^2'};
end

% External schema field names deliberately follow the portable JSON contract.
strProvenance = struct('geometry_sha256', '', ...
                       'descriptor_sha256', '', ...
                       'generator', 'MATLAB numeric payload', ...
                       'self_shadowing', false, ...
                       'shadow_samples', 0, ...
                       'ray_offset_m', 0);
if isfield(strTable, 'strJsonProvenance')
    strProvenance = strTable.strJsonProvenance;
else
    if isfield(strTable, 'charGeometrySha256')
        strProvenance.geometry_sha256 = strTable.charGeometrySha256;
    end
    if isfield(strTable, 'bSelfShadowing')
        strProvenance.self_shadowing = strTable.bSelfShadowing;
        strProvenance.shadow_samples = double(strTable.ui32SamplesPerFace);
        strProvenance.ray_offset_m = strTable.dRayOffset;
    end
end

% Assemble the artifact and validate it with the shared importer checks
strArtifact = struct('schema', 'SimulationGears.SrpResponseLut.v1', ...
                     'axis_units', 'deg', ...
                     'frame', 'SCB', ...
                     'direction', 'spacecraft_to_sun', ...
                     'layout', 'azimuth_elevation_channel', ...
                     'area_units', 'm^2', ...
                     'channels', {cellChannels}, ...
                     'channel_units', {cellUnits}, ...
                     'reference_area_m2', strTable.dReferenceArea_m2, ...
                     'azimuth', reshape(strTable.dAzimuth(1:dAzimuthCount), 1, []), ...
                     'elevation', reshape(strTable.dElevation(1:dElevationCount), 1, []), ...
                     'samples', reshape(dSamples, 1, []), ...
                     'provenance', strProvenance);
ValidateSrpLutJsonArtifact_(strArtifact);

% Check the full write, including errors reported when closing buffered data.
charJson = jsonencode(strArtifact);
dFileId = fopen(charOutputPath, 'w', 'n', 'UTF-8');
assert(dFileId >= 0, 'SrpLutJson:Write', 'Cannot open output JSON: %s', charOutputPath);
objCleanup = onCleanup(@() CloseFile_(dFileId));
dWritten = fprintf(dFileId, '%s', charJson);
assert(dWritten == numel(unicode2native(charJson, 'UTF-8')), 'SrpLutJson:Write', 'Incomplete JSON write.');
assert(fclose(dFileId) == 0, 'SrpLutJson:Write', 'Cannot close output JSON.');
end

function CloseFile_(dFileId)
%% SIGNATURE
% CloseFile_(dFileId)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Close an owned file only if it remains open after normal or exceptional exit.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dFileId Owned file identifier (MATLAB fopen returns double).
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2026  Pietro Califano  Add exception-safe output cleanup.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% fopen, fclose.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dFileId (1,1) double
end
if ~isempty(fopen(dFileId))
    fclose(dFileId);
end
end
