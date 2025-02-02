function [outputArg1,outputArg2] = Visualize3dShapeModel(strShapeModel, dPointsPositionsGT_TB, dCameraPosition_TB, dSunPosition_TB)
arguments
    strShapeModel
    dPointsPositionsGT_TB
    dCameraPosition_TB
    dSunPosition_TB = [0;0;0]
end

objFigPointCloud = figure('Renderer', 'opengl');

% Plot the mesh using patch
patch('Vertices', strShapeModel.dVerticesPos', 'Faces', strShapeModel.ui32triangVertexPtr', ...
      'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 1);
hold on
axis equal
lighting gouraud;
camlight('headlight');
objPointCloud_GT = plot3(dPointsPositionsGT_TB(1, :), dPointsPositionsGT_TB(2, :), ...
    dPointsPositionsGT_TB(3,:), 'g.', 'MarkerSize', 4);
DefaultPlotOpts()
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
view(-dCameraPosition_TB); % Camera direction % TODO (PC) use this when showing plots of emulator!

% Plot sun direction (TODO use this representation for the Sun in SLAM plots, much better)
if any(dSunPosition_TB > 0)
    hold on;
    lineScale = 1.5;
    plot3([0, lineScale * dSunPosition_TB(1)], ...
        [0, lineScale * dSunPosition_TB(2)], ...
        [0, lineScale * dSunPosition_TB(3)], 'r-', 'LineWidth', 2);
    legend('Point Cloud', 'Sun Direction');
    hold off
end
end
