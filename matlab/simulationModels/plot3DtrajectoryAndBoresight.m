function [o_dSunPhaseAngle] = plot3DtrajectoryAndBoresight(i_drCam_IN, dCamBoresightDirArray, i_drSun_IN, i_dRbody, delta_idT)
arguments
    i_drCam_IN               (3, :)
    dCamBoresightDirArray    (3, :)
    i_drSun_IN               (3, :)
    i_dRbody                 (1, 1)
    delta_idT                (1, 1) = 5
end
%% PROTOTYPE
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-04-2024        Pietro Califano        Prototype function adapted from script.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Add assert checks for input consistency
% -------------------------------------------------------------------------------------------------------------
%% Function code

ui16Nposes = size(i_drCam_IN, 2);
o_dSunPhaseAngle = zeros(ui16Nposes, 1);
cmapSun = autumn(ui16Nposes);
cmapCam = jet(ui16Nposes);

for idt = 1:delta_idT:ui16Nposes
   
    % DEVNOTE: this is equivalent to inverting the quaternion! --> TBC: Blender wants the quaternion
    % corresponding to the DCM from IN to CAMERA not the opposite (what I was generating and passing to it)
    % Namely, a LEFT-Handed QUATERNION (Hamilton)!

    Dir_D1toSun = i_drSun_IN(:, idt)./norm(i_drSun_IN(:, idt));

    % Compute Sun Phase angle to check illumination
    o_dSunPhaseAngle(idt) = acosd( dot(i_drCam_IN(1:3, idt)./norm(i_drCam_IN(1:3, idt)), Dir_D1toSun) );

    % Recompute camera boresight to check correct pointing
    dCamBoresightDir = dCamBoresightDirArray(:, idt);

    figure(1);
    hold on;
    b = gca; legend(b,'off');

    posNormTmp = norm(i_drCam_IN(1:3, idt));

    % Define vectors to plot
    i_dArrowLength = 0.5*i_dRbody;
    Zvec = (i_dArrowLength + posNormTmp/2) * dCamBoresightDir;

    % quiver3(0, 0, 0, Dir_D1toSun(1), Dir_D1toSun(2), Dir_D1toSun(3), (i_dArrowLength + posNormTmp/2),...
    %     'Color', cmapSun(idt, :), 'DisplayName', 'Sun Dir.', 'LineWidth', 1.05);

    quiver3(0, 0, 0, Dir_D1toSun(1), Dir_D1toSun(2), Dir_D1toSun(3), (i_dArrowLength + posNormTmp/2),...
        'Color', cmapSun(idt, :), 'LineWidth', 1.05);

 
    % Plot vectors
    % quiver3(i_drCam_IN(1, idt), i_drCam_IN(2, idt), i_drCam_IN(3, idt), ...
    %     Zvec(1), Zvec(2), Zvec(3), 1, 'DisplayName', 'Cam. Z', 'LineWidth', 1.05, ...
    %     'Color', cmapCam(idt,:));


    quiver3(i_drCam_IN(1, idt), i_drCam_IN(2, idt), i_drCam_IN(3, idt), ...
        Zvec(1), Zvec(2), Zvec(3), 1, 'LineWidth', 1.05, ...
        'Color', cmapCam(idt,:));

    plot3(i_drCam_IN(1, idt), i_drCam_IN(2, idt), i_drCam_IN(3, idt), 'k*', ...
        'LineStyle', 'none', 'MarkerSize', 5, 'LineWidth', 1.3)

end




end
