% TEST SETUP
% Call parent script to get data
testSimulationSetup();
% [dPointsPositionsGT_TB_check, ui32pointsIDs_check] = objTargetEmulator.GetPointsInTargetFrame(ui32pointsIDs, false);
[dPosVector_W, dRot3_WfromTB] = objTargetEmulator.GetPose();

% Assign data for test

dCameraPosition_TB = [1000; 0; 0];
dSunPositionScaled_TB   = [980; 600; 0];
dSunPosition_TB   = 1e6 * dSunPositionScaled_TB;

ui32MaxNumPoints = 5000;
ui32pointsIDs = ui32pointsIDs(1:ui32MaxNumPoints);
dPointsPositionsGT_TB = dPointsPositionsGT_TB(:,1:ui32MaxNumPoints);

% dDCM_INfromCAM (NOTE: IN is used here for legacy reasons)
objAttitudePointGeneratior = CAttitudePointingGenerator(dCameraPosition_TB, ...
                                                        dPosVector_W, ...
                                                        dSunPosition_TB);
[objAttitudePointGeneratior, dDCM_INfromCAM] = objAttitudePointGeneratior.pointToTarget("dAuxiliaryAxis", [1;0;0]);
 
strShapeModel.ui32triangVertexPtr = uint32(strShapeModel.ui32triangVertexPtr);
strShapeModel = orderfields(strShapeModel);

strTargetBodyData.strShapeModel = strShapeModel;
strTargetBodyData.dDCM_INfromTB = dRot3_WfromTB;

strTargetBodyData = orderfields(strTargetBodyData);

% Camera (object and struct)
dFocalLength = 1E3;
ui32OpticalCentre_uv = [512, 512];
ui32ImageSize = [1024, 1024];

objCameraIntrinsics = CCameraIntrinsics(dFocalLength, ui32OpticalCentre_uv, ui32ImageSize);  
disp(objCameraIntrinsics)

% Define camera object
objCamera = CProjectiveCamera(objCameraIntrinsics);

strCameraData.dDCM_INfromCAM = dDCM_INfromCAM;
strCameraData.dPosition_IN = dCameraPosition_TB; % TO MODIFY
strCameraData.dResX = ui32ImageSize(1);
strCameraData.dResY = ui32ImageSize(2);
strCameraData.dKcam = objCameraIntrinsics.K;
strCameraData       = orderfields(strCameraData);


% Assign data for CheckLMvisibility_rayTrace_MEX (LEGACY)
dFovHW = objCameraIntrinsics.GetFovHW;

strCamera.dFovX             = dFovHW(2); % [rad]
strCamera.dFovY             = dFovHW(1); % [rad]
% strCamera.dQuat_INfromCAM   = DCM2quat(dDCM_INfromCAM, true);
strCamera.bIS_JPL_QUAT      = true;
strCamera.dCAMpos_IN        = dCameraPosition_TB;
strCamera = orderfields(strCamera);


% Determines max angle between Sun direction and LM direction --> for LOCAL PHASE ANGLE CHECK
strVisibilityCheckOptions.dIllumAngleThr = deg2rad(90); % [rad] % Lower value, more permissible
% Determines max angle between -los and LM direction in TF. Angle to assign is between +Los and the
strVisibilityCheckOptions.dLosAngleThr = deg2rad(10); % [rad] % Lower value, more permissible
strVisibilityCheckOptions = orderfields(strVisibilityCheckOptions);

strFcnOptions.dIllumAngleThr                = strVisibilityCheckOptions.dIllumAngleThr;
strFcnOptions.dLosAngleThr                  = strVisibilityCheckOptions.dLosAngleThr;
strFcnOptions.bENABLE_HEUR_GEOM_FEAS_CHECK  = true;

strFcnOptions.bTwoSidedTest = true;
strFcnOptions.bPointsAreMeshVertices = true;

strTargetBody = objTargetEmulator.getTargetStruct();

%% legacyTest_CheckLMvisibility_rayTrace_MEX
% Legacy function for equivalence test (MEX because MATLAB version is too slow)
% MEx equivalence test and timing
% strTargetBody = orderfields(strTargetBody);
% strCamera = orderfields(strCamera);
% strVisibilityCheckOptions = orderfields(strVisibilityCheckOptions);
% 
% fcnHandle_LEGACY = @() CheckLMvisibility_rayTrace_MEX([double(ui32pointsIDs); dPointsPositionsGT_TB], ...
%     strTargetBody, strCamera, dSunPosition_TB./norm(dSunPosition_TB), strVisibilityCheckOptions);
% 
% [dAvgRunTime_LEGACY, dTimings_LEGACY] = AverageFunctionTiming(fcnHandle_LEGACY, 5);

%% test_RayTracePointVisibilityLocalPA_MEX

bDEBUG_MODE = false;
strTargetBodyData = orderfields(strTargetBodyData);
strCameraData = orderfields(strCameraData);
strFcnOptions = orderfields(strFcnOptions);

fcnHandle_mex = @() RayTracePointVisibility_EllipsLocalPA_MEX(uint32(ui32pointsIDs), dPointsPositionsGT_TB, ...
    strTargetBodyData, strCameraData, dSunPosition_TB, strFcnOptions, bDEBUG_MODE);

[dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle_mex, 1);

% Get output for visualization
profile('clear')
profile('on')
[bAllPointsVisibilityMask_legacyEllipsLocalPA, ~] = RayTracePointVisibility_EllipsLocalPA_MEX(uint32(ui32pointsIDs), ...
                                                                                                 dPointsPositionsGT_TB, ...
                                                                                                 strTargetBodyData, ...
                                                                                                 strCameraData, ...
                                                                                                 dSunPosition_TB, ...
                                                                                                 strFcnOptions, ...
                                                                                                 bDEBUG_MODE);
p1 = profile('info');
% profile viewer

%% test_RayTracePointVisibility_ShadowRays
bTwoSidedTest = true;
bPointsAreMeshVertices = true;
bSkipIlluminationCheck = false;

profile('clear')
profile('off')
tic
[bAllPointsVisibilityMask_RTwithShadowRays, ~] = RayTracePointVisibility_ShadowRays_MEX(uint32(ui32pointsIDs), ...
                                                                                         dPointsPositionsGT_TB, ...
                                                                                         strTargetBodyData, ...
                                                                                         strCameraData, ...
                                                                                         dSunPosition_TB, ...
                                                                                         bDEBUG_MODE, ...
                                                                                         bTwoSidedTest, ...
                                                                                         bPointsAreMeshVertices, ...
                                                                                         bSkipIlluminationCheck); 
try
    p2 = profile('info');
catch
end
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DEBUG EMULATOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % PASSED, result is identical.
% objEmulatorSettings_MaxFidelity = SFrontEndTrackerEmulatorSettings("enumRandProcessType", "NONE", ...
%     "enumTrackLossModel", "NONE", "bMaxFidelity", true);
% 
% % % Setup frontend emulator object 
% objFrontEndEmulator_MaxFidelity = CFrontEndTracker_Emulator(objCameraIntrinsics, ...
%                                                             objTargetEmulator, ...
%                                                             objEmulatorSettings_MaxFidelity);
% 
% % Run frame acquisition to get visible points
% objCameraPose_W = SPose3(strCameraData.dPosition_IN, strCameraData.dDCM_INfromCAM);
% 
% objFrontEndEmulator_MaxFidelity = objFrontEndEmulator_MaxFidelity.acquireFrame(objCameraPose_W, ...
%                                                                                 dSunPosition_TB, ...
%                                                                                 0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% test_ParallelRayTracePointVisibility_ShadowRays
% NOTE: this is essentially useless...
% profile('clear')
% profile('on')
% [bAllPointsVisibilityMask_ParallelRTwithShadowRays, dProjectedPoints_UV] = ParallelRayTracePointVisibility_ShadowRays(uint32(ui32pointsIDs), ...
%                                                                                                                         dPointsPositionsGT_TB, ...
%                                                                                                                         strTargetBodyData, ...
%                                                                                                                         strCameraData, ...
%                                                                                                                         dSunPosition_TB, ...
%                                                                                                                         bDEBUG_MODE, ...
%                                                                                                                         bTwoSidedTest); 
% p3 = profile('info');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOTS
%%%%%%%%%%%%%%%%%
% 3D plot
% Set flag for background color
bUseBlackBackground = true; % Set to false for white background

objFigPointCloud = figure('Renderer','opengl');

% Set background color based on flag

if bUseBlackBackground == true
    set(gca, 'Color', 'k'); % Axes background
    set(gcf, 'Color', 'k'); % Figure background
    charTextColor = 'w'; % White text
else
    set(gca, 'Color', 'w'); % White background
    set(gcf, 'Color', 'w');
    charTextColor = 'k'; % Black text
end

% Plot the mesh using patch
objPatchModel = patch('Vertices', strShapeModel.dVerticesPos', 'Faces', strShapeModel.ui32triangVertexPtr', ...
      'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 1, 'DisplayName', '3D model');
hold on
axis equal
lighting gouraud;
camlight('headlight');

objPointCloud_GT = plot3(dPointsPositionsGT_TB(1, :), dPointsPositionsGT_TB(2, :), ...
    dPointsPositionsGT_TB(3,:), 'g.', 'MarkerSize', 4, 'DisplayName', 'All GT points');

DefaultPlotOpts()
grid off
xlabel('X [m]', 'Color', charTextColor);
ylabel('Y [m]', 'Color', charTextColor);
zlabel('Z [m]', 'Color', charTextColor);
set(gca, 'XColor', charTextColor, 'YColor', charTextColor, 'ZColor', charTextColor);

camproj('perspective'); % Use perspective projection
campos(dCameraPosition_TB'); % Set camera to camera position
camtarget(-dCameraPosition_TB')
% view(dCameraPosition_TB'); % Camera direction % TODO (PC) use this when showing plots of emulator!

% TODO (img like visualization
% objFigImgLike = figure('Renderer','opengl');
% Project points 

% Plot sun direction (TODO use this representation for the Sun in SLAM plots, much better)
hold on;
lineScale = 1.5; 
objDirToSun = plot3([0, lineScale * dSunPositionScaled_TB(1)], ...
                    [0, lineScale * dSunPositionScaled_TB(2)], ...
                    [0, lineScale * dSunPositionScaled_TB(3)], 'r-', ...
                    'LineWidth', 2, 'DisplayName', 'To Sun');


% Show visible points in 3D plot
figure(objFigPointCloud);
hold on;
% Local phase angle check (ellipsoid assumption)
dPointsPositionsGT_RTwithEllipsLocalPA = dPointsPositionsGT_TB(:, bAllPointsVisibilityMask_legacyEllipsLocalPA);
objPointCloud_RTwithEllipsLocalPA = plot3(dPointsPositionsGT_RTwithEllipsLocalPA(1, :), dPointsPositionsGT_RTwithEllipsLocalPA(2, :), ...
                dPointsPositionsGT_RTwithEllipsLocalPA(3,:), 'b.', 'MarkerSize', 5, 'DisplayName', 'RT + Ellips. Local PA');

dPointsPositionsGT_RTwithShadowRays = dPointsPositionsGT_TB(:, bAllPointsVisibilityMask_RTwithShadowRays);
objPointCloud_RTwithShadowRays = plot3(dPointsPositionsGT_RTwithShadowRays(1, :), dPointsPositionsGT_RTwithShadowRays(2, :), ...
                dPointsPositionsGT_RTwithShadowRays(3,:), '.', 'Color', '#FFA500', 'MarkerSize', 5, 'DisplayName', 'RT + Shadow ray');

% Add legend
legend([objPatchModel, objPointCloud_GT, objDirToSun, objPointCloud_RTwithEllipsLocalPA, objPointCloud_RTwithShadowRays], 'TextColor', charTextColor);
hold off;

%%%%%%%%%%%%%%%%%
% 2D Image projection plot
[dProjectedPoints_UV]   = pinholeProjectArrayHP_DCM(objCameraIntrinsics.K, dDCM_INfromCAM', dCameraPosition_TB, dPointsPositionsGT_TB);
bPointWithinFoV = ((dProjectedPoints_UV(1, :) > 0 & dProjectedPoints_UV(1, :) < objCameraIntrinsics.ImageSize(1)) &...
                   (dProjectedPoints_UV(2, :) > 0 & dProjectedPoints_UV(2, :) < objCameraIntrinsics.ImageSize(2)))';

% Determine visible and illuminated points
bVisiblePoints_EllipsLocalPA    = bPointWithinFoV & bAllPointsVisibilityMask_legacyEllipsLocalPA;
bVisiblePoints_RTwithShadowRays = bPointWithinFoV & bAllPointsVisibilityMask_RTwithShadowRays;

% Plot projected points
objFigImgPlane = figure('Renderer','opengl'); %#ok<*FGREN>
hold on;
xlabel('U [pixels]', 'Color', 'w'); % White labels
ylabel('V [pixels]', 'Color', 'w');
title('Projected 2D Mesh in Image Plane', 'Color', 'w');

% ---- Set Background to Black ----
set(gca, 'Color', 'k'); % Axes background
set(gcf, 'Color', 'k'); % Full figure background

title('Projected 2D visible points in Image Plane');

% Plot only visible points
objVisibleProj_EllipsLocalPA = scatter(dProjectedPoints_UV(1, bVisiblePoints_EllipsLocalPA), ...
                                    dProjectedPoints_UV(2, bVisiblePoints_EllipsLocalPA), 5, ...
    'b', 'filled', 'DisplayName', 'RT + Ellips. Local PA');
hold on
objVisibleProj_ShadowRays = plot(dProjectedPoints_UV(1, bVisiblePoints_RTwithShadowRays), dProjectedPoints_UV(2, bVisiblePoints_RTwithShadowRays), ...
    '.', 'Color', '#FFA500', 'DisplayName', 'RT + Shadow Rays', 'LineStyle', 'none');

% Flip Y-axis (Image coordinates have origin at top-left)
set(gca, 'YDir', 'reverse');
set(gca, 'XColor', 'w', 'YColor', 'w'); % White axis lines

% Detector rectangle (sensor boundaries)
% Draw the detector rectangle using `rectangle()`
objCamDetector = rectangle('Position', [0, 0, objCameraIntrinsics.ImageSize(1), objCameraIntrinsics.ImageSize(2)], 'EdgeColor', 'r', 'LineWidth', 2);

% Get valid indices for 2D patch
dVertices2D = dProjectedPoints_UV';
ui32Faces2D = strShapeModel.ui32triangVertexPtr';

ui32MaxIndex = size(dVertices2D, 1);
bValidFaces = all(ui32Faces2D <= ui32MaxIndex, 2);
ui32Faces2D = ui32Faces2D(bValidFaces, :);

% Plot the projected mesh using patch
% objMeshProj = patch('Vertices', dVertices2D, 'Faces', ui32Faces2D, ...
%       'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

objMeshProj = scatter(dProjectedPoints_UV(1, :), dProjectedPoints_UV(2, :), 8, 'w', 'filled', 'DisplayName', 'Mesh vertices projection');
uistack(objMeshProj, 'bottom'); % Send the mesh to the back

legend([objVisibleProj_EllipsLocalPA, objVisibleProj_ShadowRays, objMeshProj], 'TextColor', 'w');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Test analogous function implementation
% 2D Image projection plot % DONE, OK
% [objSceneFig, cellPlotObjs] = CPointProjectionPlotter.PlotProjectedPoints({dProjectedPoints_UV, dProjectedPoints_UV(:, bVisiblePoints_EllipsLocalPA), dProjectedPoints_UV(:, bVisiblePoints_RTwithShadowRays)}, ...
%                                                             objCameraIntrisics, ...
%                                                             "bUseBlackBackground", true, ...
%                                                             "cellPlotNames", {'Mesh vertices projection', 'RT + Ellips. Local PA', 'RT + Shadow Rays'} );

