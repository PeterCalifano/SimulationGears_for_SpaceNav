function [objFig, cellFramesAxesGlobal] = PlotSceneFrames_DCM(dSceneEntityOriginArray_RenderFrame, ...
                                                            dSceneEntityDCMarray_RenderFrameFromTF, ...
                                                            dCameraOrigin_RenderFrame, ...
                                                            dCameraDCM_RenderFrameFromCam, ...
                                                            kwargs)
arguments
    dSceneEntityOriginArray_RenderFrame     (3,:) double {ismatrix, isnumeric}
    dSceneEntityDCMarray_RenderFrameFromTF  (3,3,:) double {ismatrix, isnumeric}
    dCameraOrigin_RenderFrame               (3,1) double {ismatrix, isnumeric}
    dCameraDCM_RenderFrameFromCam           (3,3,:) double {ismatrix, isnumeric}
end
arguments
    kwargs.cellPlotColors       (1,:) cell = {};
    kwargs.cellPlotNames        (1,:) cell = {};
    kwargs.charFigTitle         (1,:) string {mustBeA(kwargs.charFigTitle, ["string", "char"])} = "Reference frames visualization"
    kwargs.objFig               (1,1) {isscalar, mustBeA(kwargs.objFig, ["double", "matlab.ui.Figure"])} = 0;
    kwargs.bUseBlackBackground  (1,1) logical {islogical, isscalar} = false;
    kwargs.bEnableLegend        (1,1) logical {isscalar, islogical} = true;
    kwargs.dAxisScale           (1,1) double {isscalar, isnumeric} = 1.0     
    kwargs.bUsePhysicalPosition (1,1) logical {islogical, isscalar} = false;
end
%% SIGNATURE
% [objFig] = PlotSceneFrames_Quat(dSceneEntityOriginArray_RenderFrame, ...
%                                          dSceneEntityQuatArray_RenderFrameFromTF, ...
%                                          dCameraOrigin_RenderFrame, ...
%                                          dCameraQuat_RenderFrameFromCam, ...
%                                          kwargs.cellPlotColors, ...      
%                                          kwargs.cellPlotNames, ...       
%                                          kwargs.charFigTitle, ...        
%                                          kwargs.objFig, ...              
%                                          kwargs.bUseBlackBackground )
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function plotting frames attached to camera and objects in a scene (single time instant). The scene is
% scaled down heuristically to avoid unreadable plots. Lights are not included in the current version.
% The function support names and colours assignment following the "rule of zero" like in C++, that is,
% either you specify all names/colours or let the function decide them for you automagically. Colours of
% cameras axes are always R-G-B, while scene objects are randomized using hsv and randperm. 
% Input data are the camera origin/DCM in a generic RenderFrame. Similarly, an array of scene objects
% can be provided (any number). Background can optionally be set to black.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% arguments
%     dSceneEntityOriginArray_RenderFrame     (3,:) double {ismatrix, isnumeric}
%     dSceneEntityDCMarray_RenderFrameFromTF  (3,3,:) double {ismatrix, isnumeric}
%     dCameraOrigin_RenderFrame               (3,1) double {ismatrix, isnumeric}
%     dCameraDCM_RenderFrameFromCam           (3,3,:) double {ismatrix, isnumeric}
% end
% arguments
%     kwargs.cellPlotColors       (1,:) cell = {};
%     kwargs.cellPlotNames        (1,:) cell = {};
%     kwargs.charFigTitle         (1,:) string {mustBeA(kwargs.charFigTitle, ["string", "char"])} = "Reference frames visualization"
%     kwargs.objFig               (1,1) {isscalar, mustBeA(kwargs.objFig, ["double", "matlab.ui.Figure"])} = 0;
%     kwargs.bUseBlackBackground  (1,1) logical {islogical, isscalar} = false;
%     kwargs.bEnableLegend        (1,1) logical {isscalar, islogical} = true;
%     kwargs.dAxisScale           (1,1) double {isscalar, isnumeric} = 1.0     
%     kwargs.bUsePhysicalPosition (1,1) logical {islogical, isscalar} = false;
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objFig               (1,1) {isscalar, mustBeA(kwargs.objFig, ["double", "matlab.ui.Figure"])} = gcf;
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-02-2025    Pietro Califano     Implemented by modifying interface of PlotSceneFrames_Quat
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code
% TODO: add optional plot of the camera using computer vision toolbox: 
% https://www.mathworks.com/help/vision/ref/plotcamera.html

% Heuristic coefficient to scale camera position
dScaleCoeff = 2 * kwargs.dAxisScale;

% Normalize scale for visualization
if not(all(dSceneEntityOriginArray_RenderFrame == 0)) % Normalize positions of bodies to unity
    dSceneEntityOriginArray_RenderFrame = dAxisScale * dSceneEntityOriginArray_RenderFrame./norm(dSceneEntityOriginArray_RenderFrame);
end

% Normalize position of camera to unity
if not(all(dCameraOrigin_RenderFrame == 0)) && not(kwargs.bUsePhysicalPosition)
    dCameraOrigin_RenderFrame = dScaleCoeff * dCameraOrigin_RenderFrame./norm(dCameraOrigin_RenderFrame);
end

% Handle empty input arguments for names
dNumOfEntities = size(dSceneEntityOriginArray_RenderFrame, 2);

if isempty(kwargs.cellPlotNames)
    kwargs.cellPlotNames = cell(1, 3 + 3 * dNumOfEntities);

    % Default names for camera axis
    kwargs.cellPlotNames(1:3) = {"X Cam", "Y Cam", "Z Cam"};

    % Default names for entries
    ui32EntityPtr = 4;
    ui32EntryNum = 1;
    for idE = 1:dNumOfEntities

        kwargs.cellPlotNames(ui32EntityPtr:ui32EntityPtr+2) = { sprintf("X obj. %d", ui32EntryNum),...
            sprintf("Y obj. %d", ui32EntryNum), ...
            sprintf("Z obj. %d", ui32EntryNum)};

        ui32EntityPtr = ui32EntityPtr + 3;
        
        if mod(idE, 3) == 0
            ui32EntryNum = ui32EntryNum + 1;
        end
    end

end

% Handle empty input arguments for colours
if isempty(kwargs.cellPlotColors)
    kwargs.cellPlotColors = cell(1, 3 + 3 * dNumOfEntities);

    % Default names for camera axis
    kwargs.cellPlotColors(1:3) = {"r", "g", "b"};

    % Generate colors using HSV color space for better distinction
    % Generates a colormap with `ui32NumOfEntities` distinct randomized colors
    dEntities_colors = hsv(1e4*dNumOfEntities);
    dEntities_colors = dEntities_colors(randperm(size(dEntities_colors, 1), 3*dNumOfEntities), :);

    % Default names for entries
    ui32EntityPtr = 4;
    ui32EntryNum = 1;

    for idE = 1:dNumOfEntities

        kwargs.cellPlotColors(ui32EntityPtr:ui32EntityPtr+2) = {(dEntities_colors(ui32EntryNum, :)), ...
                                                                (dEntities_colors(ui32EntryNum + 1, :)), ...
                                                                (dEntities_colors(ui32EntryNum + 2, :))}; 
        ui32EntityPtr = ui32EntityPtr + 3;
        if mod(idE, 3) == 0
            ui32EntryNum = ui32EntryNum + 1;
        end
    end

end


% Get figure and properties
if kwargs.objFig == 0
    objFig = figure('Renderer', 'opengl');
    kwargs.bEnforcePlotOpts = true; % No figure provided, enable plot opts
    [~, charTextColor, ~] = DefaultPlotOpts(objFig, "charRenderer", "opengl", "bUseBlackBackground", kwargs.bUseBlackBackground);
else
    objFig = kwargs.objFig;
    charTextColor = objFig.Color;
end

% Plot camera frame
[cellCameraAxes, dCamBoresightaxisVec] = PlotFrameFromDCM(dCameraOrigin_RenderFrame, ...
                                                           dCameraDCM_RenderFrameFromCam, ...
                                                           kwargs.cellPlotColors(1:3), ...
                                                           kwargs.cellPlotNames(1:3), ...
                                                           objFig, ...
                                                           "dAxisScale", kwargs.dAxisScale);

% Set view along camera boresight
view(-[dCamBoresightaxisVec(1), dCamBoresightaxisVec(2), dCamBoresightaxisVec(3)]);

% Define cell of plot objects for legend
if kwargs.bEnableLegend
    % If legend is enabled, first get objects already inserted
    % TODO: get previous legend entries if any
    % cellPrevLegendObjEntries; % TODO
    disp('TODO')
    cellPrevLegendObjEntries = {};
else
    cellPrevLegendObjEntries = {};
end

% Plot scene entities frames
cellFramesAxesGlobal = cell(1, 3*dNumOfEntities + 3);

% Append camera axes to cell
cellFramesAxesGlobal(1:3) = cellCameraAxes(:);

ui32EntityPtr = 4;

for idE = 1:dNumOfEntities

    % Plot idE frame
    [cellFrameAxes] = PlotFrameFromDCM(dSceneEntityOriginArray_RenderFrame(:, idE), ...
                                       dSceneEntityDCMarray_RenderFrameFromTF(:,:, idE), ...
                                       kwargs.cellPlotColors(ui32EntityPtr:ui32EntityPtr+2), ...
                                       kwargs.cellPlotNames(ui32EntityPtr:ui32EntityPtr+2), ...
                                       objFig, ...
                                       "dAxisScale", kwargs.dAxisScale);

    % Group all scene entities axes frames into one single cell
    cellFramesAxesGlobal(ui32EntityPtr:ui32EntityPtr+2) = cellFrameAxes(:);
    ui32EntityPtr = ui32EntityPtr + 3;

end

% Additional option: plot dashed line connecting camera to any of the bodies
% TODO (PC)


% Apply additional formatting 
cellFramesAxesGlobal = [cellPrevLegendObjEntries(:)', cellFramesAxesGlobal(:)']; % Appned all objects into common cell

if not(isempty(cellFramesAxesGlobal)) && kwargs.bEnableLegend
    % Add legend if not empty
    legend([cellFramesAxesGlobal{:}], 'TextColor', charTextColor);
end

if kwargs.objFig == 0
    xlabel('X [-]');
    ylabel('Y [-]');
    zlabel('Z [-]');

    title(kwargs.charFigTitle);
    hold off;
    pause(0.05);
end

end


