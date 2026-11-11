function strTable = ValidateSrpLutJsonArtifact_(strArtifact)
%% SIGNATURE
% strTable = ValidateSrpLutJsonArtifact_(strArtifact)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate the shared JSON boundary and convert channel-fast samples to MATLAB
% arrays. The helper is host-only; numerical runtime kernels do not parse JSON.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strArtifact Decoded or prepared portable artifact.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strTable Validated compact SI force samples.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-10-2026  Pietro Califano  Centralize MATLAB interchange validation.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; move to srp/lut/private.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    strArtifact (1,1) struct
end
arguments (Output)
    strTable (1,1) struct
end

% Require every field and the v1 schema, units, frame, direction and layout
cellRequired = {'schema', 'axis_units', 'frame', 'direction', 'layout', 'area_units', ...
    'channels', 'channel_units', 'reference_area_m2', 'azimuth', 'elevation', 'samples', 'provenance'};
assert(all(isfield(strArtifact, cellRequired)), 'SrpLutJson:Schema', 'Missing required JSON fields.');
assert(strcmp(strArtifact.schema, 'SimulationGears.SrpResponseLut.v1') && ...
    strcmp(strArtifact.axis_units, 'deg') && strcmp(strArtifact.frame, 'SCB') && ...
    strcmp(strArtifact.direction, 'spacecraft_to_sun') && ...
    strcmp(strArtifact.layout, 'azimuth_elevation_channel') && strcmp(strArtifact.area_units, 'm^2'), ...
    'SrpLutJson:Schema', 'Unsupported schema, units, frame or layout.');

% Require the canonical channel names and units
assert(iscellstr(strArtifact.channels) && isvector(strArtifact.channels) && ...
    iscellstr(strArtifact.channel_units) && isvector(strArtifact.channel_units), ...
    'SrpLutJson:Channels', 'Channel names and units must be JSON string arrays.');
cellChannels = reshape(strArtifact.channels, 1, []);
cellUnits = reshape(strArtifact.channel_units, 1, []);
assert((isequal(cellChannels, {'Cr'}) && isequal(cellUnits, {'1'})) || ...
    (isequal(cellChannels, {'Cr', 'Tx', 'Ty', 'Tz'}) && isequal(cellUnits, {'1', 'm^2', 'm^2', 'm^2'})), ...
    'SrpLutJson:Channels', 'Require scalar or scalar/transverse channels in canonical order.');

% Require complete, increasing and uniform full-sphere axes
dAzimuth = reshape(strArtifact.azimuth, 1, []);
dElevation = reshape(strArtifact.elevation, 1, []);
validateattributes(dAzimuth, {'double'}, {'real', 'finite', 'vector', 'increasing', 'numel', numel(dAzimuth)});
validateattributes(dElevation, {'double'}, {'real', 'finite', 'vector', 'increasing'});
assert(numel(dAzimuth) >= 3 && numel(dElevation) >= 3 && ...
    dAzimuth(1) == -180 && dAzimuth(end) == 180 && dElevation(1) == -90 && dElevation(end) == 90, ...
    'SrpLutJson:Axes', 'Require complete full-sphere axes with endpoints.');
dAzimuthExpected = -180 + (0:numel(dAzimuth)-1)*(360/(numel(dAzimuth)-1));
dElevationExpected = -90 + (0:numel(dElevation)-1)*(180/(numel(dElevation)-1));
assert(max(abs(dAzimuth-dAzimuthExpected)) <= 1e-10 && ...
    max(abs(dElevation-dElevationExpected)) <= 1e-10, ...
    'SrpLutJson:Axes', 'Require uniform axes.');

% Require a positive area, the sample count, nonnegative Cr and exact seam and poles
validateattributes(strArtifact.reference_area_m2, {'double'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(strArtifact.samples, {'double'}, {'real', 'finite', 'vector', ...
    'numel', numel(cellChannels)*numel(dElevation)*numel(dAzimuth)});
dSamples = reshape(strArtifact.samples, numel(cellChannels), numel(dElevation), numel(dAzimuth));
assert(all(dSamples(1, :, :) >= 0, 'all'), 'SrpLutJson:Values', 'Cr must be nonnegative.');
assert(isequal(dSamples(:, :, 1), dSamples(:, :, end)) && ...
    isequal(dSamples(:, 1, :), repmat(dSamples(:, 1, 1), 1, 1, numel(dAzimuth))) && ...
    isequal(dSamples(:, end, :), repmat(dSamples(:, end, 1), 1, 1, numel(dAzimuth))), ...
    'SrpLutJson:Boundaries', 'Seam and pole values must agree exactly.');

% Nodal T is transverse; interpolation-time reprojection remains a kernel responsibility.
strTable = struct('dAzimuth', dAzimuth, ...
                  'dElevation', dElevation, ...
                  'dEffectiveCr', reshape(dSamples(1, :, :), numel(dElevation), numel(dAzimuth)), ...
                  'dReferenceArea_m2', strArtifact.reference_area_m2);
if numel(cellChannels) == 4
    dTransverse = reshape(dSamples(2:4, :, :), 3, []);
    [dAzimuthGrid, dElevationGrid] = meshgrid(dAzimuth, dElevation);
    dDirections = [cosd(dElevationGrid(:)).'.*cosd(dAzimuthGrid(:)).'; ...
        cosd(dElevationGrid(:)).'.*sind(dAzimuthGrid(:)).'; sind(dElevationGrid(:)).'];
    assert(all(abs(sum(dDirections.*dTransverse, 1)) <= 1e-10*max(1, max(abs(dTransverse), [], 1))), ...
        'SrpLutJson:Transverse', 'Nodal transverse samples must be perpendicular to Sun direction.');
    strTable.dTransverseForcePerPressure = dSamples(2:4, :, :);
end

% Require complete provenance with string identifiers and a uint32 sample count
strProvenance = strArtifact.provenance;
assert(isstruct(strProvenance) && isscalar(strProvenance) && all(isfield(strProvenance, ...
    {'geometry_sha256', 'descriptor_sha256', 'generator', 'self_shadowing', 'shadow_samples', 'ray_offset_m'})), ...
    'SrpLutJson:Provenance', 'Require complete construction provenance.');
validateattributes(strProvenance.self_shadowing, {'logical'}, {'scalar'});
cellTextFields = {'geometry_sha256', 'descriptor_sha256', 'generator'};
for ui32FieldIdx = uint32(1):uint32(numel(cellTextFields))
    charValue = strProvenance.(cellTextFields{ui32FieldIdx});
    assert((ischar(charValue) && (isrow(charValue) || isempty(charValue))) || ...
        (isstring(charValue) && isscalar(charValue) && ~ismissing(charValue)), ...
        'SrpLutJson:Provenance', 'Provenance source identifiers must be scalar strings.');
end
assert(isa(strProvenance.shadow_samples, 'double') && isreal(strProvenance.shadow_samples) && ...
    isscalar(strProvenance.shadow_samples) && isfinite(strProvenance.shadow_samples) && ...
    strProvenance.shadow_samples >= 0 && strProvenance.shadow_samples <= double(intmax('uint32')) && ...
    strProvenance.shadow_samples == fix(strProvenance.shadow_samples), ...
    'SrpLutJson:Provenance', 'Shadow sample count must fit an unsigned 32-bit integer.');
validateattributes(strProvenance.ray_offset_m, {'double'}, {'scalar', 'nonnegative', 'finite'});
assert(~strProvenance.self_shadowing || ...
    (strProvenance.shadow_samples > 0 && strProvenance.ray_offset_m > 0), ...
    'SrpLutJson:Provenance', 'Shadowing requires sample count and positive offset.');
strTable.strJsonProvenance = strProvenance;
end
