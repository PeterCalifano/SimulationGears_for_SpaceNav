function strSegmentedSpk = ExtractSegmentedSpkManoeuvres( ...
    charSpkFile, i32BodyId, i32ObserverId, charFrame, kwargs)
%% SIGNATURE
% strSegmentedSpk = ExtractSegmentedSpkManoeuvres( ...
%     charSpkFile, i32BodyId, i32ObserverId, charFrame, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Read ordered coverage arcs from one spacecraft SPK and derive impulsive
% manoeuvres from the relative states on either side of each short coverage
% gap. The caller must load its selected ancillary bundle and this spacecraft
% SPK into the SPICE pool before calling; this function does not change that
% pool. States and delta-V use km and km/s in the requested frame.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charSpkFile                   Existing spacecraft SPK path.
% i32BodyId                     Spacecraft NAIF ID in the SPK.
% i32ObserverId                 Relative-state observer NAIF ID.
% charFrame                     SPICE output frame, for example J2000.
% kwargs.ui32ExpectedArcCount    Required number of coverage arcs; zero accepts any count.
% kwargs.dMaximumGap             Largest gap treated as an impulsive boundary [s].
% kwargs.dMaximumPositionJump    Largest pre/post position difference [km].
% kwargs.dMinimumBurnMagnitude   Smallest accepted boundary delta-V [km/s].
% kwargs.dExpectedCenterId        Optional SPK segment center NAIF ID.
% kwargs.charExpectedSegmentFrame Optional SPK segment frame name.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strSegmentedSpk               Ordered 2-by-N arc bounds, segment center/frame
%                               and SPK type IDs, 6-by-(N-1) pre/post states, and
%                               3-by-(N-1) delta-V. Burn timestamps are the
%                               starts of new arcs.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-09-2026  Pietro Califano, Codex gpt-6  Extract strict generic SPK arc-boundary impulses.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MICE cspice_spkcov, cspice_kinfo, cspice_spksfs, cspice_dafus,
% cspice_spkezr; caller-owned SPICE pool.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    charSpkFile (1, :) char {mustBeFile}
    i32BodyId (1, 1) int32
    i32ObserverId (1, 1) int32
    charFrame (1, :) char {mustBeNonzeroLengthText}
    kwargs.ui32ExpectedArcCount (1, 1) uint32 = uint32(0)
    kwargs.dMaximumGap (1, 1) double {mustBePositive, mustBeFinite} = 1.0e-3
    kwargs.dMaximumPositionJump (1, 1) double {mustBeNonnegative, mustBeFinite} = 1.0e-6
    kwargs.dMinimumBurnMagnitude (1, 1) double {mustBeNonnegative, mustBeFinite} = 1.0e-12
    kwargs.dExpectedCenterId (1, 1) double = NaN
    kwargs.charExpectedSegmentFrame (1, :) char = ''
end
arguments (Output)
    strSegmentedSpk (1, 1) struct
end

% Match the loaded file handle to the named source so overlapping trajectory
% kernels cannot silently replace the requested arc states.
[charLoadedType, ~, i32ExpectedHandle, bSpkLoaded] = cspice_kinfo(charSpkFile);
if ~bSpkLoaded || ~strcmpi(charLoadedType, 'SPK')
    error('ExtractSegmentedSpkManoeuvres:SourceNotLoaded', ...
        'The selected spacecraft SPK must be loaded by the caller.');
end
if isfinite(kwargs.dExpectedCenterId) && ...
        (kwargs.dExpectedCenterId ~= fix(kwargs.dExpectedCenterId) || ...
         kwargs.dExpectedCenterId < double(intmin('int32')) || ...
         kwargs.dExpectedCenterId > double(intmax('int32')))
    error('ExtractSegmentedSpkManoeuvres:InvalidExpectedCenter', ...
        'Expected SPK segment center must be a signed NAIF integer.');
end
i32ExpectedFrameId = int32(0);
if ~isempty(kwargs.charExpectedSegmentFrame)
    i32ExpectedFrameId = cspice_namfrm(kwargs.charExpectedSegmentFrame);
    if i32ExpectedFrameId == 0
        error('ExtractSegmentedSpkManoeuvres:InvalidExpectedFrame', ...
            'Expected SPK segment frame is not known to SPICE.');
    end
end

% Read coverage from the named SPK so pool priority cannot select another
% spacecraft trajectory as the source of arc bounds.
dCoverage = cspice_spkcov(charSpkFile, i32BodyId, 10000);
if isempty(dCoverage) || mod(numel(dCoverage), 2) ~= 0
    error('ExtractSegmentedSpkManoeuvres:InvalidCoverage', ...
        'The selected spacecraft SPK has no valid coverage intervals.');
end
dArcBounds = reshape(double(dCoverage), 2, []);
ui32ArcCount = uint32(size(dArcBounds, 2));
if ui32ArcCount < uint32(2) || ...
        (kwargs.ui32ExpectedArcCount > 0 && ui32ArcCount ~= kwargs.ui32ExpectedArcCount)
    error('ExtractSegmentedSpkManoeuvres:UnexpectedArcCount', ...
        'Expected %u coverage arcs; the selected SPK contains %u.', ...
        kwargs.ui32ExpectedArcCount, ui32ArcCount);
end
if any(~isfinite(dArcBounds), 'all') || any(dArcBounds(2, :) <= dArcBounds(1, :))
    error('ExtractSegmentedSpkManoeuvres:InvalidCoverage', ...
        'SPK coverage arcs must have finite, increasing endpoints.');
end

% Reject nonpositive gaps and missing trajectory spans that cannot represent
% instantaneous manoeuvres within the caller's maximum gap.
dGaps = dArcBounds(1, 2:end) - dArcBounds(2, 1:end-1);
if any(dGaps <= 0) || any(dGaps > kwargs.dMaximumGap)
    error('ExtractSegmentedSpkManoeuvres:InvalidBoundaryGap', ...
        'Adjacent SPK arcs must have positive gaps no larger than %.9g s.', ...
        kwargs.dMaximumGap);
end

ui32BurnCount = ui32ArcCount - uint32(1);
i32ArcCenterIds = zeros(1, ui32ArcCount, 'int32');
i32ArcFrameIds = zeros(1, ui32ArcCount, 'int32');
i32ArcSegmentTypes = zeros(1, ui32ArcCount, 'int32');

% Inspect the selected segment inside each covered arc. Reject a mixed
% center/frame source or a higher-priority overlapping spacecraft kernel.
for ui32ArcIdx = uint32(1):ui32ArcCount
    dMidEpoch = mean(dArcBounds(:, ui32ArcIdx));
    [i32Handle, dDescriptor, ~, bFound] = cspice_spksfs(i32BodyId, dMidEpoch);
    if ~bFound || i32Handle ~= i32ExpectedHandle
        error('ExtractSegmentedSpkManoeuvres:AmbiguousSource', ...
            'The active spacecraft state does not come from the selected SPK.');
    end
    [~, i32DescriptorIds] = cspice_dafus(dDescriptor, int32(2), int32(6));
    i32ArcCenterIds(ui32ArcIdx) = i32DescriptorIds(2);
    i32ArcFrameIds(ui32ArcIdx) = i32DescriptorIds(3);
    i32ArcSegmentTypes(ui32ArcIdx) = i32DescriptorIds(4);
end
if any(i32ArcCenterIds ~= i32ArcCenterIds(1)) || ...
        any(i32ArcFrameIds ~= i32ArcFrameIds(1)) || ...
        any(i32ArcSegmentTypes ~= i32ArcSegmentTypes(1)) || ...
        (isfinite(kwargs.dExpectedCenterId) && ...
         any(double(i32ArcCenterIds) ~= kwargs.dExpectedCenterId)) || ...
        (i32ExpectedFrameId ~= 0 && any(i32ArcFrameIds ~= i32ExpectedFrameId))
    error('ExtractSegmentedSpkManoeuvres:SegmentMetadataMismatch', ...
        'SPK arcs have mixed types or center/frame metadata differs.');
end

dBurnPreEpochs = dArcBounds(2, 1:end-1);
dBurnPostEpochs = dArcBounds(1, 2:end);
dBurnPreStates = zeros(6, ui32BurnCount);
dBurnPostStates = zeros(6, ui32BurnCount);
dBurnDeltaV = zeros(3, ui32BurnCount);
charBodyId = sprintf('%d', i32BodyId);
charObserverId = sprintf('%d', i32ObserverId);

% Query both sides at actual covered endpoints. The new arc starts at the
% scheduled burn ET, so post-burn reference queries remain inside coverage.
for ui32BurnIdx = uint32(1):ui32BurnCount
    [i32PreHandle, ~, ~, bPreFound] = cspice_spksfs( ...
        i32BodyId, dBurnPreEpochs(ui32BurnIdx));
    [i32PostHandle, ~, ~, bPostFound] = cspice_spksfs( ...
        i32BodyId, dBurnPostEpochs(ui32BurnIdx));
    if ~bPreFound || ~bPostFound || ...
            i32PreHandle ~= i32ExpectedHandle || ...
            i32PostHandle ~= i32ExpectedHandle
        error('ExtractSegmentedSpkManoeuvres:AmbiguousBoundary', ...
            'Boundary state resolution did not select the source SPK.');
    end
    dBurnPreStates(:, ui32BurnIdx) = cspice_spkezr( ...
        charBodyId, dBurnPreEpochs(ui32BurnIdx), charFrame, 'NONE', charObserverId);
    dBurnPostStates(:, ui32BurnIdx) = cspice_spkezr( ...
        charBodyId, dBurnPostEpochs(ui32BurnIdx), charFrame, 'NONE', charObserverId);
    dBurnDeltaV(:, ui32BurnIdx) = ...
        dBurnPostStates(4:6, ui32BurnIdx) - dBurnPreStates(4:6, ui32BurnIdx);
end

dPositionJumps = vecnorm(dBurnPostStates(1:3, :) - dBurnPreStates(1:3, :), 2, 1);
dBurnMagnitudes = vecnorm(dBurnDeltaV, 2, 1);
if any(~isfinite(dBurnPreStates), 'all') || any(~isfinite(dBurnPostStates), 'all') || ...
        any(dPositionJumps > kwargs.dMaximumPositionJump)
    error('ExtractSegmentedSpkManoeuvres:PositionDiscontinuity', ...
        'Boundary states are nonfinite or exceed %.9g km position continuity.', ...
        kwargs.dMaximumPositionJump);
end
if any(dBurnMagnitudes <= kwargs.dMinimumBurnMagnitude)
    error('ExtractSegmentedSpkManoeuvres:MissingBurn', ...
        'An SPK arc boundary has no resolvable impulsive delta-V.');
end

strSegmentedSpk = struct( ...
    'dArcBounds', dArcBounds, ...
    'i32ArcCenterIds', i32ArcCenterIds, ...
    'i32ArcFrameIds', i32ArcFrameIds, ...
    'i32ArcSegmentTypes', i32ArcSegmentTypes, ...
    'dBurnPreEpochs', dBurnPreEpochs, ...
    'dBurnPostEpochs', dBurnPostEpochs, ...
    'dBurnTimestamps', dBurnPostEpochs, ...
    'dBurnPreStates', dBurnPreStates, ...
    'dBurnPostStates', dBurnPostStates, ...
    'dBurnDeltaV', dBurnDeltaV, ...
    'dBurnMagnitudes', dBurnMagnitudes);
end
