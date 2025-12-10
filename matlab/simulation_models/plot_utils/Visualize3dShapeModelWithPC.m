function [objFig, cellPlotObjs] = Visualize3dShapeModelWithPC(strShapeModel, ...
                                                              dCameraPosition_NavFrame, ...
                                                              dSunPosition_NavFrame, ...
                                                              dBodyDCM_NavFrameFromOF, ...
                                                              kwargs)
arguments (Input) % Positional
    strShapeModel % dVerticesPos, ui32triangVertexPtr
    dCameraPosition_NavFrame    (3,1) double {mustBeVector, mustBeNumeric} = [0;0;0] 
    dSunPosition_NavFrame       (3,1) double {mustBeVector, mustBeNumeric} = [0;0;0]
    dBodyDCM_NavFrameFromOF     (3,3) double {mustBeNumeric} = eye(3);
end
arguments (Input)
    kwargs.objFig           = 0
    kwargs.bUseBlackBackground          (1,1) logical {mustBeScalarOrEmpty} = false;
    kwargs.dPointsPositions_NavFrame    (3,:) double  {mustBeNumeric} = [];
    kwargs.charFigureRenderer           (1,:) string  {mustBeA(kwargs.charFigureRenderer, ["string", "char"]), ...
                                                  mustBeMember(kwargs.charFigureRenderer, ["opengl", "painters"])} = "opengl"
    kwargs.charPointsDisplayName        (1,:) string  {mustBeA(kwargs.charPointsDisplayName, ["string", "char"])} = "Points"
    kwargs.charPointsDisplayColor       (1,:) string  {mustBeA(kwargs.charPointsDisplayColor, ["string", "char"])} = "#FFDC00"
    kwargs.charDistanceUnit             (1,:) string  {mustBeA(kwargs.charDistanceUnit, ["string", "char"])} = "m"
    kwargs.bEnforcePlotOpts             (1,1) logical {mustBeScalarOrEmpty} = false
    kwargs.bUsePerspectiveView          (1,1) logical {mustBeScalarOrEmpty} = false;
    kwargs.bEnableLegend                (1,1) logical {mustBeScalarOrEmpty} = true;
    kwargs.dFaceAlpha                   (1,1) double = 1.0;
    kwargs.charPathDisplayName          (1,:) string  {mustBeA(kwargs.charPathDisplayName, ["string", "char"])} = '3D mesh model';
    kwargs.charPatchFaceColor           (1,:) = "#A9A9A9";
    kwargs.bShowAsWireframe             (1,1) logical = true;
    kwargs.bShowMeshModel               (1,1) logical = true;
end
%% SIGNATURE
% [objFig, cellPlotObjs] = Visualize3dShapeModelWithPC(strShapeModel, ...
%                                                               dCameraPosition_NavFrame, ...
%                                                               dSunPosition_NavFrame, ...
%                                                               dBodyDCM_NavFrameFromOF, ...
%                                                               kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function creating 3D visualization of shape model and input point cloud with respect to a generic world
% frame (NavFrame). The function takes care of defining figure, settings and legend objects depending on the
% inputs (kwargs).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strShapeModel
% dCameraPosition_NavFrame  = [0;0;0]
% dSunPosition_NavFrame     = [0;0;0]
% dBodyDCM_NavFrameFromOF   = eye(3);
% kwargs.objFig           = 0
% kwargs.bUseBlackBackground       (1,1) logical = false;
% kwargs.dPointsPositions_NavFrame (3, :) double = [];
% kwargs.charDistanceUnit          (1,:) string {mustBeA(kwargs.charDistanceUnit, ["string", "char"])} = "m"
% kwargs.bEnforcePlotOpts          (1,1) logical = false
% kwargs.bUsePerspectiveView       (1,1) logical = false;
% kwargs.bEnableLegend             (1,1) logical = true;
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objFig
% cellPlotObjs
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-02-2025    Pietro Califano     First implementation from script code.
% 11-02-2025    Pietro Califano     Complete release version.
% 12-02-2025    Pietro Califano     Minor update to allow usage without camera position input.
% 09-12-2025    Pietro Califano     Upgrade to use figure and axes handles
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Get figure and properties
if kwargs.objFig == 0
    objFig = figure();
    kwargs.bEnforcePlotOpts = true; % No figure provided, enable plot opts
    [~, charTextColor, ~] = DefaultPlotOpts(objFig, ...
                            "charRenderer", kwargs.charFigureRenderer, ...
                            "bUseBlackBackground", kwargs.bUseBlackBackground);
    % Create new axis
    objSceneAx = axes(objFig);
else
    objFig = kwargs.objFig;
    if kwargs.bUseBlackBackground
        charTextColor = "k";
    else
        charTextColor = "w"; 
    end
    % Get axes
    assert(isvalid(objFig), 'ERROR: figure handle is invalid!')
    objSceneAx = get(objFig, "CurrentAxes");
    if isa(objSceneAx, "matlab.graphics.GraphicsPlaceholder")
        objSceneAx = axes(objFig);
    end
end

% Assert handle validity
assert(isvalid(objSceneAx) && isa(objSceneAx, "matlab.graphics.axis.Axes"), ...
    'ERROR: axes handle is invalid or is not a matlab.graphics.axis.Axes!')


% Handle default case for shape only
if nargin < 2    
    % Set a dummy camera position for the plot to be visually good looking using 1.25 the max distance of model vertices
    dAxisLim = 1.5 * max(vecnorm(strShapeModel.dVerticesPos, 2, 1), [], 'all');
    dCameraPosition_NavFrame = dAxisLim * ones(3,1);
    % Set perspective flag to zero 
    kwargs.bUsePerspectiveView = false;
end


% Define cell of plot objects for legend
if kwargs.bEnableLegend
    % If legend is enabled, first get objects already inserted
    % TODO: get previous legend entries if any
    warning('Branch bEnableLegend = true not yet completed! cellPlotObjs defaulted to empty {}')
    cellPlotObjs = {};

else
    % Else, store objects for external use
    cellPlotObjs = {};
end


% Rotate vertices if body attitude is not eye(3)
assert(isfield(strShapeModel, "dVerticesPos") || isfield(strShapeModel, "ui32triangVertexPtr"), ...
    "Input strShapeModel must have a field 'dVerticesPos' of size [3,Nv], and a field 'ui32triangVertexPtr' of sixze [3,Nt].")

if dBodyDCM_NavFrameFromOF == eye(3)
    dVerticesPos = strShapeModel.dVerticesPos';
else
    dVerticesPos = (dBodyDCM_NavFrameFromOF * strShapeModel.dVerticesPos)';
end

% Plot the mesh using patch
hold(objSceneAx, "on");

if kwargs.bShowMeshModel
    if kwargs.bShowAsWireframe
        % Apply color modifiers to show edges only
        charEdgeColor = "#A9A9A9";
        kwargs.charPatchFaceColor = "none";
    else
        charEdgeColor = "none";
    end
    
    if size(strShapeModel.ui32triangVertexPtr, 2) > 1e4 && kwargs.bShowAsWireframe 

        ui32SubSampleIndices = uint32(ceil( linspace(1, size(strShapeModel.ui32triangVertexPtr, 2), 1e4) ));
        % Select a subset of faces
        ui32SubSampledTriangleFaces = strShapeModel.ui32triangVertexPtr(:, ui32SubSampleIndices)';

    else
        ui32SubSampledTriangleFaces = strShapeModel.ui32triangVertexPtr';
    end

    objShadedMeshPlot = patch(objSceneAx, 'Vertices', dVerticesPos, ...
                                'Faces', ui32SubSampledTriangleFaces, ...
                                'FaceColor', kwargs.charPatchFaceColor, ...
                                'EdgeColor', charEdgeColor, ...
                                'FaceAlpha', kwargs.dFaceAlpha, ...
                                'DisplayName', kwargs.charPathDisplayName, ...
                                "LineWidth", 0.1, "LineStyle", "-.");

    hold(objSceneAx, "on")
    lighting(objSceneAx, "gouraud");
    camlight(objSceneAx, "headlight", "local");
    % Append object to cell
    cellPlotObjs = [cellPlotObjs(:)', {objShadedMeshPlot}];
end

if not(isempty(kwargs.dPointsPositions_NavFrame))

    objPointCloudPlot = plot3(objSceneAx, ...
                              kwargs.dPointsPositions_NavFrame(1, :), ...
                              kwargs.dPointsPositions_NavFrame(2, :), ...
                              kwargs.dPointsPositions_NavFrame(3,:), ...
                              '.', 'MarkerSize', 7, ...
                              "Color", kwargs.charPointsDisplayColor, ...
                              "DisplayName", kwargs.charPointsDisplayName, ...
                              "LineStyle", "none");

    % Append object to cell
    objPointCloudPlot = {objPointCloudPlot};
    cellPlotObjs = [cellPlotObjs(:), objPointCloudPlot(:)];
end

if kwargs.bEnforcePlotOpts

    [~, charTextColor, ~] = DefaultPlotOpts(objFig, ...
                        "charRenderer", kwargs.charFigureRenderer, ...
                        "bUseBlackBackground", kwargs.bUseBlackBackground);

    axis(objSceneAx, "equal");

    if nargin < 2 && kwargs.objFig == 0 % Keep axis limits if there is an input figure!
        xlim(objSceneAx, [-dAxisLim, dAxisLim])
        ylim(objSceneAx, [-dAxisLim, dAxisLim])
        zlim(objSceneAx, [-dAxisLim, dAxisLim])
    end

    % DEVNOTE: these are already in default plot opts if used
    xlabel(objSceneAx, sprintf('X [%s]', kwargs.charDistanceUnit), 'Color', charTextColor)
    ylabel(objSceneAx, sprintf('Y [%s]', kwargs.charDistanceUnit), 'Color', charTextColor)
    zlabel(objSceneAx, sprintf('Z [%s]', kwargs.charDistanceUnit), 'Color', charTextColor)
    set(objSceneAx, 'XColor', charTextColor, 'YColor', charTextColor, 'ZColor', charTextColor);

    campos(objSceneAx, dCameraPosition_NavFrame'); % Set camera to camera position

    if not(kwargs.bUsePerspectiveView)
        view(objSceneAx, -dCameraPosition_NavFrame); % Camera direction % TODO (PC) use this when showing plots of emulator!
    else
        % TODO: clarify what is this visualization and if useful
        camproj(objSceneAx, 'perspective'); % Use perspective projection
        camtarget(objSceneAx, - transpose(dCameraPosition_NavFrame));
    end
end

% Plot sun direction if provided
if any(dSunPosition_NavFrame > 0)

    hold(objSceneAx, "on");
    % If Sun specified, move light there
    objLight = light(objSceneAx, "Style", "Infinite", "Position", dSunPosition_NavFrame);
    camlight(objLight);

    % Normalize position to avoid unreadable plots
    dScaleDistanceSun = norm(dCameraPosition_NavFrame);
    dSunPosition_NavFrame = dScaleDistanceSun * dSunPosition_NavFrame./norm(dSunPosition_NavFrame);

    
    dLineScale = 0.7;
    objSunDirPlot = plot3(objSceneAx, [0, dLineScale * dSunPosition_NavFrame(1)], ...
                                    [0, dLineScale * dSunPosition_NavFrame(2)], ...
                                    [0, dLineScale * dSunPosition_NavFrame(3)], ...
                                    '-', 'Color', '#f4ce37', 'LineWidth', 1.25, 'DisplayName', 'To Sun');
    axis(objSceneAx, "equal");

    % Append object to cell
    objSunDirPlot = {objSunDirPlot};
    cellPlotObjs = [cellPlotObjs(:)', objSunDirPlot(:)];

end


if not(isempty(cellPlotObjs)) && kwargs.bEnableLegend
    % Add legend if not empty
    legend(objSceneAx, [cellPlotObjs{:}], ...
            'TextColor', charTextColor);
elseif isempty(cellPlotObjs) && kwargs.bEnableLegend
    warning('%s status: No object to show in figure.', char(mfilename) );
end


end
