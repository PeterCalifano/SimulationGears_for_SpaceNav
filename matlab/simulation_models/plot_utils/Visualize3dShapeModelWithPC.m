function [objFig, cellPlotObjs] = Visualize3dShapeModelWithPC(strShapeModel, ...
                                                              dCameraPosition_NavFrame, ...
                                                              dSunPosition_NavFrame, ...
                                                              dBodyDCM_NavFrameFromOF, ...
                                                              kwargs)
arguments (Input) % Positional
    strShapeModel % dVerticesPos, ui32triangVertexPtr
    dCameraPosition_NavFrame % TODO: allow function to work WITHOUT DEFINING CAMERA
    dSunPosition_NavFrame   = [0;0;0]
    dBodyDCM_NavFrameFromOF = eye(3);
end
arguments (Input)
    kwargs.objFig           = 0
    kwargs.bUseBlackBackground       (1,1) logical {islogical, isscalar} = false;
    kwargs.dPointsPositions_NavFrame (3, :) double = [];
    kwargs.charDistanceUnit          (1,:) string {mustBeA(kwargs.charDistanceUnit, ["string", "char"])} = "m"
    kwargs.bEnforcePlotOpts          (1,1) logical {isscalar, islogical} = false
    kwargs.bUsePerspectiveView       (1,1) logical {isscalar, islogical} = false;
    kwargs.bEnableLegend             (1,1) logical {isscalar, islogical} = true;
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
% arguments (Input) % Positional
%     strShapeModel
%     dCameraPosition_NavFrame
%     dSunPosition_NavFrame   = [0;0;0]
%     dBodyDCM_NavFrameFromOF = eye(3);
% end
% arguments (Input)
%     kwargs.objFig           = 0
%     kwargs.bUseBlackBackground       (1,1) logical {islogical, isscalar} = false;
%     kwargs.dPointsPositions_NavFrame (3, :) double = [];
%     kwargs.charDistanceUnit          (1,:) string {mustBeA(kwargs.charDistanceUnit, ["string", "char"])} = "m"
%     kwargs.bEnforcePlotOpts          (1,1) logical {isscalar, islogical} = false
%     kwargs.bUsePerspectiveView       (1,1) logical {isscalar, islogical} = false;
%     kwargs.bEnableLegend             (1,1) logical {isscalar, islogical} = true;
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objFig
% cellPlotObjs
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-02-2025        Pietro Califano         First implementation from script code.
% 11-02-2025        Pietro Califano         Complete release version.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% DefaultPlotOpts()
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Get figure and properties
if kwargs.objFig == 0
    objFig = figure('Renderer', 'opengl');
    kwargs.bEnforcePlotOpts = true; % No figure provided, enable plot opts
    [~, charTextColor, ~] = DefaultPlotOpts(objFig, "charRenderer", "opengl", "bUseBlackBackground", kwargs.bUseBlackBackground);
else
    objFig = kwargs.objFig;
    charTextColor = objFig.Color;
end

% Refocus figure and hold
figure(objFig)
hold on;

% Define cell of plot objects for legend
if kwargs.bEnableLegend
    % If legend is enabled, first get objects already inserted
    % TODO: get previous legend entries if any
else
    % Else, store objects for external use
    cellPlotObjs = {};
end


% Rotate vertices if body attitude is not eye(3)
if dBodyDCM_NavFrameFromOF == eye(3)
    dVerticesPos = strShapeModel.dVerticesPos';
else
    dVerticesPos = (dBodyDCM_NavFrameFromOF * strShapeModel.dVerticesPos)';
end

% Plot the mesh using patch
objShadedMeshPlot = patch('Vertices', dVerticesPos, 'Faces', strShapeModel.ui32triangVertexPtr', ...
                    'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 1, 'DisplayName', '3D mesh model');
hold on
lighting gouraud;
camlight('headlight');

% Append object to cell
cellPlotObjs = [cellPlotObjs(:)', {objShadedMeshPlot}];

if not(isempty(kwargs.dPointsPositions_NavFrame))

    objPointCloudPlot = plot3(kwargs.dPointsPositions_NavFrame(1, :), ...
                              kwargs.dPointsPositions_NavFrame(2, :), ...
                              kwargs.dPointsPositions_NavFrame(3,:), ...
                              'g.', 'MarkerSize', 6, 'DisplayName', '');

    % Append object to cell
    objPointCloudPlot = {objPointCloudPlot};
    cellPlotObjs = [cellPlotObjs(:), objPointCloudPlot(:)];
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
    
    % Normalize position to avoid unreadable plots
    dScaleDistanceSun = norm(dCameraPosition_NavFrame);
    dSunPosition_NavFrame = dScaleDistanceSun * dSunPosition_NavFrame./norm(dSunPosition_NavFrame);

    hold on;
    dLineScale = 1.2;
    objSunDirPlot = plot3([0, dLineScale * dSunPosition_NavFrame(1)], ...
                    [0, dLineScale * dSunPosition_NavFrame(2)], ...
                    [0, dLineScale * dSunPosition_NavFrame(3)], ...
                    '-', 'Color', '#f48037', 'LineWidth', 2, 'DisplayName', 'To Sun');
    axis equal
    % Append object to cell
    objSunDirPlot = {objSunDirPlot};
    cellPlotObjs = [cellPlotObjs(:)', objSunDirPlot(:)];

end


if not(isempty(cellPlotObjs)) && kwargs.bEnableLegend
    % Add legend if not empty
    legend([cellPlotObjs{:}], ...
            'TextColor', charTextColor);
end


end
