function DemoAttitudePointingGenerator()
%% SIGNATURE
% DemoAttitudePointingGenerator()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Visualize deterministic camera locations and attitude frames generated for
% target pointing with seeded boresight roll and off-pointing. This example is
% intentionally interactive; automated geometric checks live in
% testAttitudePointingGenerator.
%
% Example:
%   DemoAttitudePointingGenerator()
%
% Expected output:
%   Figures showing the camera-position shell, generated attitude frames, and
%   the seeded off-pointing-angle distribution.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 22-07-2026    Pietro Califano, Codex     Extract the visual workflow from the automated test suite.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CAttitudePointingGenerator
% -------------------------------------------------------------------------------------------------------------

charExampleDirectory = fileparts(mfilename('fullpath'));
charRepositoryRoot = fileparts(fileparts(fileparts(charExampleDirectory)));
run(fullfile(charRepositoryRoot, 'matlab', 'SetupSimGears.m'));

% Place cameras on a deterministic wavy ring around one target so the
% generated batch is visually diverse without an external pose sampler.
ui32NumPoses = uint32(24);
dAzimuthAngles = linspace(0.0, 2.0 * pi, double(ui32NumPoses) + 1);
dAzimuthAngles(end) = [];
dElevationAngles = deg2rad(20.0 .* sin(2.0 .* dAzimuthAngles));
dCameraRange = 2.0e3;
dCameraPosition_Frame = dCameraRange .* [ ...
    cos(dElevationAngles) .* cos(dAzimuthAngles); ...
    cos(dElevationAngles) .* sin(dAzimuthAngles); ...
    sin(dElevationAngles)];
dTargetPosition_Frame = zeros(3, 1);
dSunPosition_Frame = [1.0e8; 2.0e8; 0.5e8];

objPositionFigure = figure('Name', 'Attitude pointing camera locations');
scatter3(dCameraPosition_Frame(1, :), ...
         dCameraPosition_Frame(2, :), ...
         dCameraPosition_Frame(3, :), ...
         30.0, 'b', 'filled');
hold on;
plot3(dTargetPosition_Frame(1), dTargetPosition_Frame(2), dTargetPosition_Frame(3), ...
      'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 12.0);
axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Camera locations around the target');
legend(objPositionFigure.CurrentAxes, {'Camera', 'Target'});

% Seed random perturbations so repeated runs remain directly comparable.
rng(42, 'twister');
objPointingGenerator = CAttitudePointingGenerator( ...
    dCameraPosition_Frame, ...
    dTargetPosition_Frame, ...
    dSunPosition_Frame, ...
    'bShowAttitudePointingPlot', true);
[~, ~, ~, dOffPointingAngles] = objPointingGenerator.pointToTarget( ...
    'dSigmaDegRotAboutBoresight', 5.0, ...
    'dSigmaOffPointingDegAngle', 2.0, ...
    'enumOffPointingMode', "randomAxis");

figure('Name', 'Attitude pointing off-pointing angles');
histogram(dOffPointingAngles, 'BinWidth', 0.2);
grid on;
xlabel('Off-pointing half-cone angle [deg]');
ylabel('Count');
title('Seeded off-pointing distribution');

end
