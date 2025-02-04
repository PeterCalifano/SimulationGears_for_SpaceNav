function PlotSceneFrames(dOriginTB_RenderFrame, ...
    dQuat_TB1fromRenderFrame, ...
    dOriginCamera_RenderFrame, ...
    dQuat_CamFromRenderFrame, ...
    ui32FigureID)
arguments
    dOriginTB_RenderFrame       (3,1)
    dQuat_TB1fromRenderFrame    (1,4)
    dOriginCamera_RenderFrame   (3,1)
    dQuat_CamFromRenderFrame    (1,4)
    ui32FigureID                (1,1) = 1
end

    % Normalize scale for visualization
    if not(all(dOriginTB_RenderFrame == 0))
        dOriginTB_RenderFrame = dOriginTB_RenderFrame./norm(dOriginTB_RenderFrame);
    end

    if not(all(dOriginCamera_RenderFrame == 0))
        dOriginCamera_RenderFrame = 2 * dOriginCamera_RenderFrame./norm(dOriginCamera_RenderFrame);
    end

    % Ensure quaternions are normalized
    dQuat_TB1fromRenderFrame = dQuat_TB1fromRenderFrame./norm(dQuat_TB1fromRenderFrame);
    dQuat_CamFromRenderFrame = dQuat_CamFromRenderFrame./norm(dQuat_CamFromRenderFrame);

    % Convert quaternions to rotation matrices
    rot_TB1fromRenderFrame = quat2rotm(dQuat_TB1fromRenderFrame);
    rot_CamFromRenderFrame = quat2rotm(dQuat_CamFromRenderFrame);

    % Frame A axes
    xA = rot_TB1fromRenderFrame(:, 1);
    yA = rot_TB1fromRenderFrame(:, 2);
    zA = rot_TB1fromRenderFrame(:, 3);

    % Frame B axes
    xB = rot_CamFromRenderFrame(:, 1);
    yB = rot_CamFromRenderFrame(:, 2);
    zB = rot_CamFromRenderFrame(:, 3);

    % Plot Frame A
    figure(ui32FigureID);
    clf; % Clear previous plots
    hold on;
    % ax_obj = gobjects(6,1);
    ax_obj1 = quiver3(dOriginTB_RenderFrame(1), dOriginTB_RenderFrame(2), dOriginTB_RenderFrame(3), xA(1), xA(2), xA(3), 1, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'X TB');
    ax_obj2 = quiver3(dOriginTB_RenderFrame(1), dOriginTB_RenderFrame(2), dOriginTB_RenderFrame(3), yA(1), yA(2), yA(3), 1, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Y TB');
    ax_obj3 = quiver3(dOriginTB_RenderFrame(1), dOriginTB_RenderFrame(2), dOriginTB_RenderFrame(3), zA(1), zA(2), zA(3), 1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Z TB');

    % Plot Frame B
    ax_obj4 = quiver3(dOriginCamera_RenderFrame(1), dOriginCamera_RenderFrame(2), dOriginCamera_RenderFrame(3), xB(1), xB(2), xB(3), 1, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'X Cam');
    ax_obj5 = quiver3(dOriginCamera_RenderFrame(1), dOriginCamera_RenderFrame(2), dOriginCamera_RenderFrame(3), yB(1), yB(2), yB(3), 1, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Y Cam');
    ax_obj6 = quiver3(dOriginCamera_RenderFrame(1), dOriginCamera_RenderFrame(2), dOriginCamera_RenderFrame(3), zB(1), zB(2), zB(3), 1, 'y', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Z Cam');
    
    view([zB(1), zB(2), zB(3)]);

    % Formatting
    legend([ax_obj1, ax_obj2, ax_obj3, ax_obj4, ax_obj5, ax_obj6]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    axis equal;
    title('Reference Frames Visualization');
    hold off;
    pause(1)
end