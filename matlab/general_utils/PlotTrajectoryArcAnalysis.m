function strPlotOutputs = PlotTrajectoryArcAnalysis(strAnalysis, options)
arguments
    strAnalysis (1,1) struct
    options.bShowFigures (1,1) logical = true
    options.charTitlePrefix (1,1) string {mustBeText} = "Trajectory arc analysis"
    options.bPlotPhaseAngleTrajectory (1,1) logical = true
end

figureVisibility = 'off';
if options.bShowFigures
    figureVisibility = 'on';
end

objTrajectoryData = struct();
objTrajectoryData.dPosSC_W = strAnalysis.dPos_W;
objTrajectoryData.dVelSC_W = strAnalysis.dVel_W;
objTrajectoryData.dTimestamps = strAnalysis.dTimestamps;
objTrajectoryData.dArcIntervals = strAnalysis.dPhaseIntervals;

objSegmentFig = figure( ...
    'Name', sprintf('%s - Segments', char(options.charTitlePrefix)), ...
    'Visible', figureVisibility, ...
    'Color', 'w');

[~, ~] = Plot3dTrajectoryArcsWithManNodes( ...
    objTrajectoryData, ...
    'objSceneFig', objSegmentFig, ...
    'bHoldOn', true, ...
    'bAxisEqual', true, ...
    'bGridOn', true, ...
    'bShowLegend', true, ...
    'bPlotOrigin', true, ...
    'bPlotBoundaryArrows', true, ...
    'dBoundaryArrowColor', [0.90, 0.15, 0.10], ...
    'cellArcLabels', strAnalysis.cellPhaseLabels, ...
    'dColors', strAnalysis.dPhaseColors);
title(gca, sprintf('%s - Segment View', char(options.charTitlePrefix)), 'Interpreter', 'none');

objPhaseTrajectoryFig = gobjects(1, 0);
if options.bPlotPhaseAngleTrajectory && ~isempty(strAnalysis.dPhaseAngleDeg)
    objPhaseTrajectoryFig = createPhaseAngleTrajectoryFigure_(strAnalysis, objTrajectoryData, figureVisibility, options.charTitlePrefix);
end

objTimelineFig = createTimelineFigure_(strAnalysis, figureVisibility, options.charTitlePrefix);

strPlotOutputs = struct();
strPlotOutputs.objSegmentFig = objSegmentFig;
strPlotOutputs.objPhaseTrajectoryFig = objPhaseTrajectoryFig;
strPlotOutputs.objTimelineFig = objTimelineFig;
end

function objFig = createPhaseAngleTrajectoryFigure_(strAnalysis, objTrajectoryData, figureVisibility, charTitlePrefix)
objFig = figure( ...
    'Name', sprintf('%s - Phase Angle', char(charTitlePrefix)), ...
    'Visible', figureVisibility, ...
    'Color', 'w');

objAxes = axes(objFig);
hold(objAxes, 'on');
grid(objAxes, 'on');
axis(objAxes, 'equal');
view(objAxes, 3);

[dArcIndexPairs, dBoundaryIdx] = resolveArcIndexPairs_(objTrajectoryData.dTimestamps, objTrajectoryData.dArcIntervals);
for idPhase = 1:size(dArcIndexPairs, 1)
    dIdx = dArcIndexPairs(idPhase, 1):dArcIndexPairs(idPhase, 2);
    surface( ...
        objAxes, ...
        [objTrajectoryData.dPosSC_W(1, dIdx); objTrajectoryData.dPosSC_W(1, dIdx)], ...
        [objTrajectoryData.dPosSC_W(2, dIdx); objTrajectoryData.dPosSC_W(2, dIdx)], ...
        [objTrajectoryData.dPosSC_W(3, dIdx); objTrajectoryData.dPosSC_W(3, dIdx)], ...
        [strAnalysis.dPhaseAngleDeg(dIdx); strAnalysis.dPhaseAngleDeg(dIdx)], ...
        'FaceColor', 'none', ...
        'EdgeColor', 'interp', ...
        'LineWidth', 2.0, ...
        'HandleVisibility', 'off');
end

plotBoundaryVelocityArrows_(objAxes, objTrajectoryData.dPosSC_W, objTrajectoryData.dVelSC_W, dBoundaryIdx);
plot3(objAxes, 0, 0, 0, 'k.', 'MarkerSize', 18, 'HandleVisibility', 'off');

colormap(objFig, turbo(256));
dColorLimits = [min(strAnalysis.dPhaseAngleDeg), max(strAnalysis.dPhaseAngleDeg)];
if dColorLimits(1) == dColorLimits(2)
    dColorLimits = dColorLimits + [-1.0, 1.0];
end
clim(objAxes, dColorLimits);
objColorbar = colorbar(objAxes);
ylabel(objColorbar, 'Phase Angle [deg]');

xlabel(objAxes, 'X [ref]');
ylabel(objAxes, 'Y [ref]');
zlabel(objAxes, 'Z [ref]');
title(objAxes, sprintf('%s - Phase Angle View', char(charTitlePrefix)), 'Interpreter', 'none');
end

function objFig = createTimelineFigure_(strAnalysis, figureVisibility, charTitlePrefix)
bHasPhaseAngles = ~isempty(strAnalysis.dPhaseAngleDeg);
dNumRows = 2 + double(bHasPhaseAngles);

objFig = figure( ...
    'Name', sprintf('%s - Timeline', char(charTitlePrefix)), ...
    'Visible', figureVisibility, ...
    'Color', 'w');

objTiles = tiledlayout(objFig, dNumRows, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axRange = nexttile(objTiles);

dRelDays = strAnalysis.dRelativeTime_s / 86400;
dRangeLimits = [min(strAnalysis.dRange), min(strAnalysis.dRange), max(strAnalysis.dRange), max(strAnalysis.dRange)];

plotTimelineBackground_(axRange, strAnalysis, dRangeLimits);
plot(axRange, dRelDays, strAnalysis.dRange, 'Color', [0.15, 0.15, 0.15], 'LineWidth', 1.4);
plotBoundaryNodes_(axRange, strAnalysis, strAnalysis.dRange);
xlabel(axRange, 'Time [days]');
ylabel(axRange, 'Range');
title(axRange, 'Range History');

if bHasPhaseAngles
    axPhase = nexttile(objTiles);
    dPhaseLimits = computePatchLimits_(strAnalysis.dPhaseAngleDeg);
    plotTimelineBackground_(axPhase, strAnalysis, dPhaseLimits);
    plot(axPhase, dRelDays, strAnalysis.dPhaseAngleDeg, 'Color', [0.15, 0.15, 0.15], 'LineWidth', 1.4);
    plotBoundaryNodes_(axPhase, strAnalysis, strAnalysis.dPhaseAngleDeg);
    xlabel(axPhase, 'Time [days]');
    ylabel(axPhase, 'Phase Angle [deg]');
    title(axPhase, 'Sun-Spacecraft Phase Angle');
end

axTiming = nexttile(objTiles);
dTimingLimits = [0.5, 0.5, numel(strAnalysis.strPhases) + 0.5, numel(strAnalysis.strPhases) + 0.5];
plotTimelineBackground_(axTiming, strAnalysis, dTimingLimits);
hold(axTiming, 'on');
grid(axTiming, 'on');
for idPhase = 1:numel(strAnalysis.strPhases)
    plot(axTiming, ...
        ([strAnalysis.strPhases(idPhase).dStartTime, strAnalysis.strPhases(idPhase).dEndTime] - strAnalysis.dTimestamps(1)) / 86400, ...
        [idPhase, idPhase], ...
        'LineWidth', 5.0, ...
        'Color', strAnalysis.dPhaseColors(idPhase, :));
end
xlabel(axTiming, 'Time [days]');
ylabel(axTiming, 'Phase');
yticks(axTiming, 1:numel(strAnalysis.strPhases));
yticklabels(axTiming, strAnalysis.cellPhaseLabels);
title(axTiming, 'Phase Timing');

title(objTiles, charTitlePrefix, 'Interpreter', 'none');
end

function plotTimelineBackground_(objAxes, strAnalysis, yPatchLimits)
hold(objAxes, 'on');
grid(objAxes, 'on');

for idPhase = 1:numel(strAnalysis.strPhases)
    dStartDay = (strAnalysis.strPhases(idPhase).dStartTime - strAnalysis.dTimestamps(1)) / 86400;
    dEndDay = (strAnalysis.strPhases(idPhase).dEndTime - strAnalysis.dTimestamps(1)) / 86400;
    patch( ...
        objAxes, ...
        [dStartDay, dEndDay, dEndDay, dStartDay], ...
        yPatchLimits, ...
        strAnalysis.dPhaseColors(idPhase, :), ...
        'FaceAlpha', 0.05, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end

for idPhase = 1:(numel(strAnalysis.strPhases) - 1)
    dBoundaryDay = (strAnalysis.strPhases(idPhase).dEndTime - strAnalysis.dTimestamps(1)) / 86400;
    xline(objAxes, dBoundaryDay, '-', 'Color', [0.30, 0.30, 0.30], 'LineWidth', 0.9, 'Alpha', 0.35, 'HandleVisibility', 'off');
end
end

function plotBoundaryNodes_(objAxes, strAnalysis, dQuantity)
for idPhase = 1:numel(strAnalysis.strPhases)
    dStartIdx = strAnalysis.strPhases(idPhase).dStartIdx;
    dEndIdx = strAnalysis.strPhases(idPhase).dEndIdx;

    plot(objAxes, ...
        strAnalysis.dRelativeTime_s(dStartIdx) / 86400, ...
        dQuantity(dStartIdx), ...
        'o', ...
        'MarkerSize', 5, ...
        'LineWidth', 1.0, ...
        'Color', strAnalysis.dPhaseColors(idPhase, :), ...
        'MarkerFaceColor', strAnalysis.dPhaseColors(idPhase, :), ...
        'HandleVisibility', 'off');

    plot(objAxes, ...
        strAnalysis.dRelativeTime_s(dEndIdx) / 86400, ...
        dQuantity(dEndIdx), ...
        'o', ...
        'MarkerSize', 5, ...
        'LineWidth', 1.0, ...
        'Color', strAnalysis.dPhaseColors(idPhase, :), ...
        'MarkerFaceColor', strAnalysis.dPhaseColors(idPhase, :), ...
        'HandleVisibility', 'off');
end
end

function [dArcIndexPairs, dBoundaryIdx] = resolveArcIndexPairs_(dTimestamps, dArcIntervals)
dArcIndexPairs = zeros(size(dArcIntervals, 1), 2);
dBoundaryIdx = zeros(1, max(size(dArcIntervals, 1) - 1, 0));
for idPhase = 1:size(dArcIntervals, 1)
    dArcIndexPairs(idPhase, 1) = closestIdx_(dTimestamps, dArcIntervals(idPhase, 1));
    dArcIndexPairs(idPhase, 2) = closestIdx_(dTimestamps, dArcIntervals(idPhase, 2));
    if idPhase > 1
        dBoundaryIdx(idPhase - 1) = dArcIndexPairs(idPhase, 1);
    end
end
end

function plotBoundaryVelocityArrows_(objAxes, dPos_W, dVel_W, dBoundaryIdx)
if isempty(dBoundaryIdx)
    return
end

dSceneExtent = max(max(dPos_W, [], 2) - min(dPos_W, [], 2));
dSceneExtent = max(dSceneExtent, eps);
dArrowScale = 0.035 * dSceneExtent;

for idBoundary = 1:numel(dBoundaryIdx)
    dIdx = dBoundaryIdx(idBoundary);
    dDir = dVel_W(:, dIdx);
    dDirNorm = norm(dDir);
    if dDirNorm < eps
        continue
    end

    dArrow = dArrowScale * dDir / dDirNorm;
    quiver3( ...
        objAxes, ...
        dPos_W(1, dIdx), ...
        dPos_W(2, dIdx), ...
        dPos_W(3, dIdx), ...
        dArrow(1), ...
        dArrow(2), ...
        dArrow(3), ...
        0, ...
        'Color', [0.90, 0.15, 0.10], ...
        'LineWidth', 1.8, ...
        'MaxHeadSize', 2.4, ...
        'HandleVisibility', 'off');
end
end

function dPatchLimits = computePatchLimits_(dValues)
dMinValue = min(dValues);
dMaxValue = max(dValues);
if dMinValue == dMaxValue
    dPatchLimits = [dMinValue - 1.0, dMinValue - 1.0, dMaxValue + 1.0, dMaxValue + 1.0];
else
    dPatchLimits = [dMinValue, dMinValue, dMaxValue, dMaxValue];
end
end

function dIdx = closestIdx_(dTimestamps, dTime)
[~, dIdx] = min(abs(dTimestamps - dTime));
end

function mustBeText(value)
if ~(ischar(value) || (isstring(value) && isscalar(value)))
    error('Expected a text scalar.');
end
end
