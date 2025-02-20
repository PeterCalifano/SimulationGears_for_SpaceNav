function [cellFrameAxes, dZaxisVec, dXaxisVec, dYaxisVec] = PlotFrameFromDCM(dEntityOrigin_RenderFrame, ...
                                                                             dEntityDCM_RenderFrameFromTF, ...
                                                                             cellPlotColors, ...
                                                                             cellPlotNames, ...
                                                                             objFig, ...
                                                                             kwargs)
arguments (Input)
    dEntityOrigin_RenderFrame       (3,1) double {isvector, isnumeric}
    dEntityDCM_RenderFrameFromTF    (3,3) double {ismatrix, isnumeric}
    cellPlotColors                  (3,1) cell    
    cellPlotNames                   (3,1) cell 
    objFig                          (1,1) {isscalar, mustBeA(objFig, ["double", "matlab.ui.Figure"])} = 0;
end
arguments (Input)
    kwargs.dAxisScale (1,1) double {isscalar, isnumeric} = 1;
end
arguments (Output)
    cellFrameAxes (1,3) cell
    dZaxisVec     (3,1) double {isvector, isnumeric}
    dXaxisVec     (3,1) double {isvector, isnumeric}
    dYaxisVec     (3,1) double {isvector, isnumeric}
end
%% SIGNATURE
% [cellFrameAxes, dZaxisVec, dXaxisVec, dYaxisVec] = PlotFrameFromDCM(dEntityOrigin_RenderFrame, ...
%                                                                              dEntityDCM_RenderFrameFromTF, ...
%                                                                              cellPlotColors, ...
%                                                                              cellPlotNames, ...
%                                                                              objFig, ...
%                                                                              kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function plotting axes of a orthonormal reference frame using quiver3 given its origin and attitude
% matrix. A new figure gets created if none is passed as input.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dEntityOrigin_RenderFrame       (3,1) double {isvector, isnumeric}
% dEntityDCM_RenderFrameFromTF    (3,3) double {ismatrix, isnumeric}
% cellPlotColors                  (3,1) cell
% cellPlotNames                   (3,1) cell
% objFig                          (1,1) {isscalar, mustBeA(objFig, ["double", "matlab.ui.Figure"])} = 0;
% kwargs.dAxisScale (1,1) double {isscalar, isnumeric} = 1;
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% cellFrameAxes (1,3) cell
% dZaxisVec     (3,1) double {isvector, isnumeric}
% dXaxisVec     (3,1) double {isvector, isnumeric}
% dYaxisVec     (3,1) double {isvector, isnumeric}
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-02-2025    Pietro Califano     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Get axis from attitude matrix
dXaxisVec = dEntityDCM_RenderFrameFromTF(1, :);
dYaxisVec = dEntityDCM_RenderFrameFromTF(2, :);
dZaxisVec = dEntityDCM_RenderFrameFromTF(3, :);

% Get current axis if 0
if objFig == 0
    objFig = gcf;
end

% Refocus figure object
figure(objFig);
hold on;
cellFrameAxes = cell(3,1);

% Plot axis of frame
cellFrameAxes{1} = quiver3(dEntityOrigin_RenderFrame(1), dEntityOrigin_RenderFrame(2), ...
    dEntityOrigin_RenderFrame(3), dXaxisVec(1), dXaxisVec(2), dXaxisVec(3), kwargs.dAxisScale, ...
    "Color", cellPlotColors{1}, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', cellPlotNames{1});
hold on;

cellFrameAxes{2} = quiver3(dEntityOrigin_RenderFrame(1), dEntityOrigin_RenderFrame(2), ...
    dEntityOrigin_RenderFrame(3), dYaxisVec(1), dYaxisVec(2), dYaxisVec(3), kwargs.dAxisScale, ...
    "Color", cellPlotColors{2}, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', cellPlotNames{2});
hold on;

cellFrameAxes{3} = quiver3(dEntityOrigin_RenderFrame(1), dEntityOrigin_RenderFrame(2), ...
    dEntityOrigin_RenderFrame(3), dZaxisVec(1), dZaxisVec(2), dZaxisVec(3), kwargs.dAxisScale, ...
    "Color", cellPlotColors{3}, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', cellPlotNames{3});

axis equal
end
