function cellTrajPlotObjs = Plot3dTrajectoryArcsWithManNodes(objReferenceMissionData, kwargs)
% Plot trajectory arcs separated by manoeuvres. Manoeuvre nodes are dots colored
% with the color of the subsequent arc.

arguments
    objReferenceMissionData (1,1) {mustBeA(objReferenceMissionData, "SReferenceImagesDataset")}
end
arguments
    kwargs.objSceneFig  = []              % figure handle or []
    kwargs.bHoldOn      (1,1) logical = true
    kwargs.bAxisEqual   (1,1) logical = true
    kwargs.bGridOn      (1,1) logical = false
end

% Figure setup
if ~isempty(kwargs.objSceneFig)
    figure(kwargs.objSceneFig);
else
    figure;
end

if kwargs.bHoldOn
    hold on; 
else
    hold off; 
end

% Extract data
dPosSC_W          = objReferenceMissionData.dPosSC_W;          % 3xN
dTimestamps       = objReferenceMissionData.dTimestamps(:)';   % 1xN
dManStartTimes    = objReferenceMissionData.dManoeuvresStartTimestamps(:)'; % 1xM or []

% Build arc breakpoints (indices in 1..N)
dN = size(dPosSC_W, 2);
if isempty(dManStartTimes)
    dBreakIdx = [1, dN];                    % single arc
    dManIdx   = [];                          % no nodes
else
    dManIdx_tmp = arrayfun(@(t) localClosestIdx(dTimestamps, t), dManStartTimes);
    dBreakIdx   = unique([1, dManIdx_tmp, dN]);   % include starts, manoeuvre instants, end
    dManIdx     = dManIdx_tmp(:)';                % keep order for node plotting
end

% Prepare colormap: one color per arc
dNumArcs = numel(dBreakIdx) - 1;
dNumArcs = max(dNumArcs, 1);
dColors  = distinguishable_colors(dNumArcs);

% Plot arcs
cellLineHandles = cell(1, dNumArcs);
cellLegendLabels = cell(1, dNumArcs);
for dArc = 1:dNumArcs
    dI0 = dBreakIdx(dArc);
    dI1 = dBreakIdx(dArc+1);
    if dI1 <= dI0, continue; end
    cellLineHandles{dArc} = plot3( ...
        dPosSC_W(1,dI0:dI1), ...
        dPosSC_W(2,dI0:dI1), ...
        dPosSC_W(3,dI0:dI1), ...
        'LineWidth', 1.5, 'Color', dColors(dArc,:));
    cellLegendLabels{dArc} = sprintf('Arc %d', dArc);
end

% Plot manoeuvre nodes colored as subsequent arcs
cellNodeHandles = {};
for dK = 1:numel(dManIdx)
    dIdx = dManIdx(dK);
    dArcNext = find(dBreakIdx(1:end-1) == dIdx, 1, 'first');
    if isempty(dArcNext)
        dArcNext = find(dBreakIdx(1:end-1) > dIdx, 1, 'first');
        if isempty(dArcNext), dArcNext = dNumArcs; end
    end
    cellNodeHandles{end+1} = plot3( ...
        dPosSC_W(1,dIdx), dPosSC_W(2,dIdx), dPosSC_W(3,dIdx), ...
        '.', 'MarkerSize', 18, 'Color', dColors(dArcNext,:)); %#ok<AGROW>
end

% Axes styling
if kwargs.bAxisEqual, axis equal; end
if kwargs.bGridOn, grid on; else, grid off; end
xlabel('X [ref]'); ylabel('Y [ref]'); zlabel('Z [ref]');

% Legend
legend([cellLineHandles{:}], cellLegendLabels, 'Location', 'best');

% Output handles
cellTrajPlotObjs = { ...
    'line_arcs',  cellLineHandles; ...
    'node_dots',  cellNodeHandles  ...
    };

end

% ---------- helpers ----------
function dIdx = localClosestIdx(dTimestamps, dTime)
    % return index of timestamp closest to dTime
    [~, dIdx] = min(abs(dTimestamps - dTime));
end
function colors = distinguishable_colors(n)

    % Generate visually distinct RGB colors

    if n > 5
        hsv_colors = hsv(n);
        colors = hsv_colors(randperm(n), :);
    else
        base = lines(max(n,7));
        colors = base(mod(0:n-1, size(base,1)) + 1, :);

    end
end
