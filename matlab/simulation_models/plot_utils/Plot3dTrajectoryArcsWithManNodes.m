function [cellTrajPlotObjs, objFig] = Plot3dTrajectoryArcsWithManNodes(objReferenceMissionData, kwargs)
%Plot3dTrajectoryArcsWithManNodes Plot selected trajectory arcs on a 3D scene.
%
% The function accepts either:
%   - a continuous trajectory with split timestamps in
%     dArcStartTimestamps / dManoeuvresStartTimestamps, or
%   - explicit arc intervals in dArcIntervals = [tStart, tEnd].
%
% Required fields in objReferenceMissionData:
%   dPosSC_W     [3xN] or [Nx3] Cartesian positions
%   dTimestamps  [1xN] or [Nx1] timestamps associated with dPosSC_W
%
% Optional fields in objReferenceMissionData:
%   dVelSC_W     [3xN] or [Nx3] Cartesian velocities used to draw
%                boundary arrows after each manoeuvre node

arguments
    objReferenceMissionData (1,1)
    kwargs.objSceneFig = []
    kwargs.bHoldOn (1,1) logical = true
    kwargs.bAxisEqual (1,1) logical = true
    kwargs.bGridOn (1,1) logical = false
    kwargs.bShowLegend (1,1) logical = true
    kwargs.bPlotOrigin (1,1) logical = true
    kwargs.bPlotBoundaryArrows (1,1) logical = false
    kwargs.charPositionUnits {mustBeTextScalar_} = "ref"
    kwargs.cellArcLabels = {}
    kwargs.dColors double = zeros(0, 3)
    kwargs.dBoundaryArrowColor (1,3) double = [0.90, 0.15, 0.10]
    kwargs.dBoundaryArrowScale (1,1) double = NaN
end

% Validate the input container and normalize the trajectory to a common
% 3xN layout before any indexing logic is applied.
validateTrajectoryInput_(objReferenceMissionData);
[dPosSC_W, dVelSC_W, dTimestamps] = normalizeTrajectoryData_(objReferenceMissionData);
[arcIndexPairs, nodeIndices, nodeColorIndices] = resolveArcIndexPairs_(objReferenceMissionData, dTimestamps);

dNumArcs = size(arcIndexPairs, 1);
dColors = resolveArcColors_(dNumArcs, kwargs.dColors);
cellArcLabels = resolveArcLabels_(dNumArcs, kwargs.cellArcLabels);

% Reuse the caller figure when provided, otherwise create a new scene.
objFig = resolveFigure_(kwargs.objSceneFig);
objAxes = resolveAxes_(objFig);

if kwargs.bHoldOn
    hold(objAxes, 'on');
else
    cla(objAxes);
    hold(objAxes, 'on');
end

% Plot each arc with its assigned color and legend label.
lineHandles = gobjects(1, dNumArcs);
for idx = 1:dNumArcs
    arcPair = arcIndexPairs(idx, :);
    lineHandles(idx) = plot3( ...
        objAxes, ...
        dPosSC_W(1, arcPair(1):arcPair(2)), ...
        dPosSC_W(2, arcPair(1):arcPair(2)), ...
        dPosSC_W(3, arcPair(1):arcPair(2)), ...
        'LineWidth', 1.5, ...
        'Color', dColors(idx, :), ...
        'DisplayName', cellArcLabels{idx});
end

% Mark each boundary with either a velocity arrow after the manoeuvre or a
% node dot, depending on the caller request.
boundaryHandles = gobjects(1, numel(nodeIndices));

if kwargs.bPlotBoundaryArrows
    if isempty(dVelSC_W)
        error('dVelSC_W must be provided when bPlotBoundaryArrows is true.');
    end

    boundaryHandles = plotBoundaryArrows_( ...
        objAxes, dPosSC_W, dVelSC_W, nodeIndices, ...
        kwargs.dBoundaryArrowColor, kwargs.dBoundaryArrowScale);
else
    for idx = 1:numel(nodeIndices)
        dNodeIndex = nodeIndices(idx);
        dColorIndex = nodeColorIndices(idx);
        boundaryHandles(idx) = plot3( ...
            objAxes, ...
            dPosSC_W(1, dNodeIndex), ...
            dPosSC_W(2, dNodeIndex), ...
            dPosSC_W(3, dNodeIndex), ...
            '.', ...
            'MarkerSize', 18, ...
            'LineWidth', 1.0, ...
            'Color', dColors(dColorIndex, :), ...
            'HandleVisibility', 'off');
    end
end

originHandle = gobjects(1, 1);

if kwargs.bPlotOrigin
    originHandle = plot3( ...
        objAxes, 0, 0, 0, ...
        'k.', ...
        'MarkerSize', 18, ...
        'HandleVisibility', 'off');
end

% Apply the requested scene formatting once all graphics objects exist.
if kwargs.bAxisEqual
    axis(objAxes, 'equal');
end

if kwargs.bGridOn
    grid(objAxes, 'on');
else
    grid(objAxes, 'off');
end

view(objAxes, 3);
xlabel(objAxes, sprintf('X [%s]', char(kwargs.charPositionUnits)));
ylabel(objAxes, sprintf('Y [%s]', char(kwargs.charPositionUnits)));
zlabel(objAxes, sprintf('Z [%s]', char(kwargs.charPositionUnits)));

if kwargs.bShowLegend && ~isempty(lineHandles)
    legend(objAxes, lineHandles, cellArcLabels, 'Location', 'best');
end

if ~kwargs.bHoldOn
    hold(objAxes, 'off');
end

cellTrajPlotObjs = { ...
    'line_arcs', lineHandles; ...
    'boundary_markers', boundaryHandles; ...
    'origin', originHandle ...
    };
end

%% Internal helper functions
function validateTrajectoryInput_(objReferenceMissionData)
if ~(isstruct(objReferenceMissionData) || isa(objReferenceMissionData, 'SReferenceImagesDataset'))
    error('objReferenceMissionData must be a struct or SReferenceImagesDataset.');
end

requiredFields = {'dPosSC_W', 'dTimestamps'};
for idx = 1:numel(requiredFields)
    if ~hasFieldOrProperty_(objReferenceMissionData, requiredFields{idx})
        error('objReferenceMissionData is missing required field %s.', requiredFields{idx});
    end
end
end

function [dPosSC_W, dVelSC_W, dTimestamps] = normalizeTrajectoryData_(objReferenceMissionData)
dPosSC_W = objReferenceMissionData.dPosSC_W;

if size(dPosSC_W, 1) ~= 3 && size(dPosSC_W, 2) == 3
    dPosSC_W = dPosSC_W';
end

if size(dPosSC_W, 1) ~= 3
    error('dPosSC_W must have size 3xN or Nx3.');
end

dVelSC_W = [];
if hasFieldOrProperty_(objReferenceMissionData, 'dVelSC_W') ...
        && ~isempty(objReferenceMissionData.dVelSC_W)
    dVelSC_W = normalizeVectorSamples_(objReferenceMissionData.dVelSC_W, 'dVelSC_W');
    if size(dVelSC_W, 2) ~= size(dPosSC_W, 2)
        error('dVelSC_W length must match the number of trajectory samples.');
    end
end

dTimestamps = objReferenceMissionData.dTimestamps(:)';
if numel(dTimestamps) ~= size(dPosSC_W, 2)
    error('dTimestamps length must match the number of trajectory samples.');
end
end

function dSamples = normalizeVectorSamples_(dSamples, charName)
if size(dSamples, 1) ~= 3 && size(dSamples, 2) == 3
    dSamples = dSamples';
end

if size(dSamples, 1) ~= 3
    error('%s must have size 3xN or Nx3.', charName);
end
end

function [arcIndexPairs, nodeIndices, nodeColorIndices] = resolveArcIndexPairs_(objReferenceMissionData, dTimestamps)
% Support either explicit arc intervals or a continuous trajectory split by boundary timestamps.
dNumSamples = numel(dTimestamps);

if isfield(objReferenceMissionData, 'dArcIntervals') && ~isempty(objReferenceMissionData.dArcIntervals)
    dArcIntervals = objReferenceMissionData.dArcIntervals;
    if size(dArcIntervals, 2) ~= 2
        error('dArcIntervals must have size Nx2.');
    end

    arcIndexPairs = zeros(size(dArcIntervals, 1), 2);
    nodeIndices = zeros(1, max(size(dArcIntervals, 1) - 1, 0));
    nodeColorIndices = zeros(1, max(size(dArcIntervals, 1) - 1, 0));
    for idx = 1:size(dArcIntervals, 1)
        dStartIdx = closestIndex_(dTimestamps, dArcIntervals(idx, 1));
        dEndIdx = closestIndex_(dTimestamps, dArcIntervals(idx, 2));
        if dEndIdx < dStartIdx
            error('Arc interval %d resolves to an invalid index pair.', idx);
        end
        arcIndexPairs(idx, :) = [dStartIdx, dEndIdx];
        if idx > 1
            nodeIndices(idx - 1) = dStartIdx;
            nodeColorIndices(idx - 1) = idx;
        end
    end
    return
end

if isfield(objReferenceMissionData, 'dArcStartTimestamps') && ~isempty(objReferenceMissionData.dArcStartTimestamps)
    dBoundaryTimestamps = objReferenceMissionData.dArcStartTimestamps(:)';
elseif isfield(objReferenceMissionData, 'dManoeuvresStartTimestamps') && ~isempty(objReferenceMissionData.dManoeuvresStartTimestamps)
    dBoundaryTimestamps = objReferenceMissionData.dManoeuvresStartTimestamps(:)';
else
    arcIndexPairs = [1, dNumSamples];
    nodeIndices = zeros(1, 0);
    nodeColorIndices = zeros(1, 0);
    return
end

dBoundaryIndices = arrayfun(@(dTime) closestIndex_(dTimestamps, dTime), dBoundaryTimestamps);
dBoundaryIndices = unique(sort(dBoundaryIndices(:)));
dBoundaryIndices = dBoundaryIndices(:)';
dBoundaryIndices = dBoundaryIndices(dBoundaryIndices > 1 & dBoundaryIndices <= dNumSamples);

dBreakIndices = [1, dBoundaryIndices, dNumSamples + 1];
arcIndexPairs = zeros(numel(dBreakIndices) - 1, 2);
for idx = 1:size(arcIndexPairs, 1)
    arcIndexPairs(idx, :) = [dBreakIndices(idx), dBreakIndices(idx + 1) - 1];
end

nodeIndices = dBoundaryIndices;
nodeColorIndices = 1:numel(dBoundaryIndices);
end

function dIndex = closestIndex_(dTimestamps, dTime)
[~, dIndex] = min(abs(dTimestamps - dTime));
end

function dColors = resolveArcColors_(dNumArcs, dRequestedColors)
if isempty(dRequestedColors)
    dColors = defaultArcColors_(dNumArcs);
    return
end

if size(dRequestedColors, 2) ~= 3 || size(dRequestedColors, 1) < dNumArcs
    error('dColors must have size Nx3 with at least one row per arc.');
end

dColors = dRequestedColors(1:dNumArcs, :);
end

function cellArcLabels = resolveArcLabels_(dNumArcs, cellRequestedLabels)
if isempty(cellRequestedLabels)
    cellArcLabels = arrayfun(@(idx) sprintf('Arc %d', idx - 1), 1:dNumArcs, 'UniformOutput', false);
    return
end

if isstring(cellRequestedLabels)
    cellRequestedLabels = cellstr(cellRequestedLabels(:));
end

if ~iscell(cellRequestedLabels) || numel(cellRequestedLabels) ~= dNumArcs
    error('cellArcLabels must contain exactly one label per arc.');
end

cellArcLabels = cellRequestedLabels(:)';
end

function dColors = defaultArcColors_(dNumArcs)
% Use a dense colormap by default so adjacent arcs remain distinguishable
% even when the number of segments grows.
dColors = turbo(max(dNumArcs, 2));
dColors = dColors(1:dNumArcs, :);
end

function boundaryHandles = plotBoundaryArrows_( ...
    objAxes, dPosSC_W, dVelSC_W, nodeIndices, dArrowColor, dRequestedArrowScale)
boundaryHandles = gobjects(1, numel(nodeIndices));

if isempty(nodeIndices)
    return
end

dSceneExtent = max(max(dPosSC_W, [], 2) - min(dPosSC_W, [], 2));
dSceneExtent = max(dSceneExtent, eps);

if isnan(dRequestedArrowScale)
    dArrowScale = 0.035 * dSceneExtent;
else
    dArrowScale = dRequestedArrowScale;
end

for idx = 1:numel(nodeIndices)
    dNodeIndex = nodeIndices(idx);
    dDirection = dVelSC_W(:, dNodeIndex);
    dDirectionNorm = norm(dDirection);

    if dDirectionNorm < eps
        continue
    end

    dArrowVector = dArrowScale * dDirection / dDirectionNorm;
    boundaryHandles(idx) = quiver3( ...
        objAxes, ...
        dPosSC_W(1, dNodeIndex), ...
        dPosSC_W(2, dNodeIndex), ...
        dPosSC_W(3, dNodeIndex), ...
        dArrowVector(1), ...
        dArrowVector(2), ...
        dArrowVector(3), ...
        0, ...
        'Color', dArrowColor, ...
        'LineWidth', 1.8, ...
        'MaxHeadSize', 2.4, ...
        'HandleVisibility', 'off');
end
end

function objFig = resolveFigure_(objSceneFig)
if isempty(objSceneFig)
    objFig = figure('Color', 'w');
    return
end

if isgraphics(objSceneFig, 'figure')
    objFig = figure(objSceneFig);
    return
end

error('objSceneFig must be empty or a valid figure handle.');
end

function objAxes = resolveAxes_(objFig)
objAxes = get(objFig, 'CurrentAxes');
if isempty(objAxes) || ~isgraphics(objAxes, 'axes')
    objAxes = axes(objFig);
end
end

function mustBeTextScalar_(value)
if ~(ischar(value) || (isstring(value) && isscalar(value)))
    error('Expected a text scalar.');
end
end

function tf = hasFieldOrProperty_(objReferenceMissionData, charName)
tf = isfield(objReferenceMissionData, charName);
if ~tf && ~isstruct(objReferenceMissionData)
    tf = isprop(objReferenceMissionData, charName);
end
end
