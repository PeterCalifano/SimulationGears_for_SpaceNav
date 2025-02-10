function [outputArg1,outputArg2] = Visualize3dShapeModelWithPC(strShapeModel, ...
                                                                dCameraPosition_NavFrame, ...
                                                                dSunPosition_NavFrame, ...
                                                                kwargs)
arguments (Input) % Positional
    strShapeModel
    dCameraPosition_NavFrame
    dSunPosition_NavFrame        = [0;0;0]
end
arguments (Input)
    kwargs.objFig           = 0
    kwargs.bUseBlackBackground
    kwargs.dPointsPositions_NavFrame (3, :) double = [];
    kwargs.charDistanceUnit          (1,:) string {mustBeA(kwargs.charDistanceUnit, ["string", "char"])} = "m"
    kwargs.bEnforcePlotOpts          (1,1) logical {isscalar, islogical} = false
    kwargs.bUsePerspectiveView       (1,1) logical {isscalar, islogical} = false;
end
%% SIGNATURE
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-02-2025        Pietro Califano         First implementation from script code.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Get figure
if kwargs.objFig == 0
    objFig = figure('Renderer','opengl');
    kwargs.bEnforcePlotOpts = true; % No figure provided, enable plot opts
else
    objFig = kwargs.objFig;
end

% Refocus figure and hold
figure(objFig)
hold on;

% Set background color based on flag
% Default option
set(gca, 'Color', 'w'); % White background
set(gcf, 'Color', 'w');
charTextColor = 'k'; % Black text

if kwargs.bUseBlackBackground == true
    set(gca, 'Color', 'k'); % Axes background
    set(gcf, 'Color', 'k'); % Figure background
    charTextColor = 'w'; % White text
end

% Define cell of plot objects for legend
cellPlotObjs = {};
% TODO: get previous legend entries if any

% Plot the mesh using patch
objShadedMeshPlot = patch('Vertices', strShapeModel.dVerticesPos', 'Faces', strShapeModel.ui32triangVertexPtr', ...
                    'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 1, 'DisplayName', '3D mesh model');
hold on
lighting gouraud;
camlight('headlight');

% Append object to cell
cellPlotObjs = {cellPlotObjs(:), objShadedMeshPlot};


if not(isempty(kwargs.dPointsPositions_NavFrame))

    objPointCloudPlot = plot3(kwargs.dPointsPositions_NavFrame(1, :), ...
                              kwargs.dPointsPositions_NavFrame(2, :), ...
                              kwargs.dPointsPositions_NavFrame(3,:), ...
                              'g.', 'MarkerSize', 6, 'DisplayName', '');

    % Append object to cell
    cellPlotObjs = {cellPlotObjs(:), objPointCloudPlot};
end

if kwargs.bEnforcePlotOpts

    DefaultPlotOpts() % TODO: use new version?

    axis equal

    % DEVNOTE: these are already in default plot opts if used
    xlabel(sprintf('X [%s]', kwargs.charDistanceUnit), 'Color', charTextColor)
    ylabel(sprintf('Y [%s]', kwargs.charDistanceUnit), 'Color', charTextColor)
    zlabel(sprintf('Z [%s]', kwargs.charDistanceUnit), 'Color', charTextColor)
    set(gca, 'XColor', charTextColor, 'YColor', charTextColor, 'ZColor', charTextColor);

    % TODO: keep this? TBC

    if not(kwargs.bUsePerspectiveView)
        view(-dCameraPosition_NavFrame); % Camera direction % TODO (PC) use this when showing plots of emulator!
    else
        % TODO: clarify what is this visualization and if useful
        camproj('perspective'); % Use perspective projection
        campos(dCameraPosition_NavFrame'); % Set camera to camera position
        camtarget(-dCameraPosition_NavFrame')
    end
end

% Plot sun direction if provided
if any(dSunPosition_NavFrame > 0)

    hold on;
    dLineScale = 1.5;
    objSunDirPlot = plot3([0, dLineScale * dSunPosition_NavFrame(1)], ...
                    [0, dLineScale * dSunPosition_NavFrame(2)], ...
                    [0, dLineScale * dSunPosition_NavFrame(3)], ...
                    'r-', 'LineWidth', 2, 'DisplayName', 'To Sun');

    % Append object to cell
    cellPlotObjs = {cellPlotObjs(:), objSunDirPlot};

end


if not(isempty(cellPlotObjs))
    % Add legend if not empty
    legend([cellPlotObjs{:}], ...
            'TextColor', charTextColor);
end


end
