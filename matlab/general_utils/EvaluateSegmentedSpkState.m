function [dRelativeState, strEvaluation] = EvaluateSegmentedSpkState( ...
    charSpkFile, i32BodyId, i32ObserverId, charFrame, ...
    dRequestedEpoch, dArcBounds, kwargs)
%% SIGNATURE
% [dRelativeState, strEvaluation] = EvaluateSegmentedSpkState( ...
%     charSpkFile, i32BodyId, i32ObserverId, charFrame, ...
%     dRequestedEpoch, dArcBounds, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate one target-relative state inside a selected segmented SPK. At an
% arc-start burn timestamp, the caller chooses the pre- or post-burn side to
% match simulation action ordering. The returned record retains the requested
% timestamp separately from the covered endpoint used for the SPICE query.
% The caller owns SPICE-pool setup and source-file integrity verification.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charSpkFile                Loaded spacecraft SPK path.
% i32BodyId                  Spacecraft NAIF ID.
% i32ObserverId              Relative-state observer NAIF ID.
% charFrame                  Output SPICE frame.
% dRequestedEpoch            Nominal reference timestamp [ET seconds].
% dArcBounds                 Validated 2-by-N arc start/end ET bounds.
% kwargs.charBoundarySide     'pre' or 'post' at an arc-start burn timestamp.
% kwargs.dBoundaryTolerance   Maximum distance from arc start treated as
%                             the burn epoch [s].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dRelativeState             6-by-1 position/velocity [km, km/s].
% strEvaluation              Source arc, boundary side, and actual query ET.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-09-2026  Pietro Califano, Codex gpt-6  Add explicit boundary-side nominal SPK evaluation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MICE cspice_kinfo, cspice_spksfs, cspice_spkezr; caller-owned SPICE pool.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    charSpkFile (1, :) char {mustBeFile}
    i32BodyId (1, 1) int32
    i32ObserverId (1, 1) int32
    charFrame (1, :) char {mustBeNonzeroLengthText}
    dRequestedEpoch (1, 1) double {mustBeFinite}
    dArcBounds (2, :) double {mustBeFinite}
    kwargs.charBoundarySide (1, :) char {mustBeMember( ...
        kwargs.charBoundarySide, {'pre', 'post'})} = 'post'
    kwargs.dBoundaryTolerance (1, 1) double {mustBeFinite, mustBeNonnegative} = 1e-6
end
arguments (Output)
    dRelativeState (6, 1) double
    strEvaluation (1, 1) struct
end

% Require an ordered set of disjoint covered arcs before selecting a state.
ui32ArcCount = uint32(size(dArcBounds, 2));
if ui32ArcCount == 0 || ...
        any(dArcBounds(2, :) <= dArcBounds(1, :)) || ...
        any(dArcBounds(1, 2:end) <= dArcBounds(2, 1:end-1))
    error('EvaluateSegmentedSpkState:InvalidArcs', ...
        'Arc bounds must contain ordered, disjoint covered intervals.');
end

% Detect an arc-start burn independently of the source gap. The caller's
% action order determines which side supplies the aligned reference state.
bBoundary = false;
ui32BoundaryIndex = uint32(0);
ui32ArcIndex = uint32(0);
dEvaluatedEpoch = dRequestedEpoch;

if ui32ArcCount > 1
    [dDistance, dClosestBoundaryIndex] = min(abs(dRequestedEpoch - dArcBounds(1, 2:end)));
    
    if dDistance <= kwargs.dBoundaryTolerance
        bBoundary = true;
        ui32BoundaryIndex = uint32(dClosestBoundaryIndex);
        if strcmp(kwargs.charBoundarySide, 'pre')
            ui32ArcIndex = ui32BoundaryIndex;
            dEvaluatedEpoch = dArcBounds(2, ui32ArcIndex);
        else
            ui32ArcIndex = ui32BoundaryIndex + uint32(1);
            dEvaluatedEpoch = dArcBounds(1, ui32ArcIndex);
        end
    end
end

% Require ordinary queries to lie within an arc; never interpolate missing
% reference data across a coverage gap.
if ~bBoundary
    dMatchingArcs = find(dRequestedEpoch >= dArcBounds(1, :) & ...
                        dRequestedEpoch <= dArcBounds(2, :));
    if numel(dMatchingArcs) ~= 1
        error('EvaluateSegmentedSpkState:OutsideCoverage', ...
            'Requested reference ET does not lie within one SPK arc.');
    end
    ui32ArcIndex = uint32(dMatchingArcs);
end

% Reject pool priority changes after condition materialization, including a
% second spacecraft SPK loaded over the selected source.
[charLoadedType, ~, i32ExpectedHandle, bSpkLoaded] = cspice_kinfo(charSpkFile);
if ~bSpkLoaded || ~strcmpi(charLoadedType, 'SPK')
    error('EvaluateSegmentedSpkState:SourceNotLoaded', ...
        'The selected spacecraft SPK must be loaded by the caller.');
end
[i32StateHandle, ~, ~, bStateFound] = ...
    cspice_spksfs(i32BodyId, dEvaluatedEpoch);
if ~bStateFound || i32StateHandle ~= i32ExpectedHandle
    error('EvaluateSegmentedSpkState:AmbiguousSource', ...
        'The active spacecraft state does not come from the selected SPK.');
end

charBodyId = sprintf('%d', i32BodyId);
charObserverId = sprintf('%d', i32ObserverId);
dRelativeState = cspice_spkezr( ...
    charBodyId, dEvaluatedEpoch, charFrame, 'NONE', charObserverId);
if any(~isfinite(dRelativeState))
    error('EvaluateSegmentedSpkState:InvalidState', ...
        'The selected SPK produced a nonfinite reference state.');
end

strEvaluation = struct( ...
    'dRequestedEpoch', dRequestedEpoch, ...
    'dEvaluatedEpoch', dEvaluatedEpoch, ...
    'ui32ArcIndex', ui32ArcIndex, ...
    'ui32BoundaryIndex', ui32BoundaryIndex, ...
    'bBoundary', bBoundary, ...
    'charBoundarySide', kwargs.charBoundarySide);
end
