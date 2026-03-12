function [] = PlotRays(dCamPosition_TB, dPointPos_TB, dIntersectPoint_TB, dLightPoint_TB)
arguments
    dCamPosition_TB
    dPointPos_TB
    dIntersectPoint_TB
    dLightPoint_TB = [0;0;0]
end

figure(55);
hold on;
plot3(dIntersectPoint_TB(1), dIntersectPoint_TB(2), dIntersectPoint_TB(3), 'b*', 'DisplayName', 'Intersect');
plot3(dCamPosition_TB(1), dCamPosition_TB(2), dCamPosition_TB(3), 'r*', 'DisplayName', 'Camera');
plot3(dPointPos_TB(1), dPointPos_TB(2), dPointPos_TB(3), 'k*', 'DisplayName', 'Point to test');

% Plot lines from camera to both points
plot3([dCamPosition_TB(1), dIntersectPoint_TB(1)], ...
    [dCamPosition_TB(2), dIntersectPoint_TB(2)], ...
    [dCamPosition_TB(3), dIntersectPoint_TB(3)], 'b-', 'DisplayName', 'Ray cam to intersection');

plot3([dCamPosition_TB(1), dPointPos_TB(1)], ...
    [dCamPosition_TB(2), dPointPos_TB(2)], ...
    [dCamPosition_TB(3), dPointPos_TB(3)], 'k-', 'DisplayName', 'Ray cam to point');

if any(dLightPoint_TB ~= 0)

    % Scale light point to avoid unreadable plot
    dLightPoint_TB = 2 * norm(dCamPosition_TB) * dLightPoint_TB./norm(dLightPoint_TB);

    plot3(dLightPoint_TB(1), dLightPoint_TB(2), dLightPoint_TB(3), '*', 'Color', '#FFA500', 'DisplayName', 'Light point (scaled)');

plot3([dLightPoint_TB(1), dPointPos_TB(1)], ...
        [dLightPoint_TB(2), dPointPos_TB(2)], ...
        [dLightPoint_TB(3), dPointPos_TB(3)], '-', 'Color', '#FFA500', 'DisplayName', 'Point to light (scaled)');

end

% legend;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3)
title('3D Plot with Camera rays');
axis equal
                
end
