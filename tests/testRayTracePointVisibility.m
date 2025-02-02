% TEST SETUP
% Call parent script to get data
testSimulationSetup();
% [dPointsPositionsGT_TB_check, ui32pointsIDs_check] = objTargetEmulator.GetPointsInTargetFrame(ui32pointsIDs, false);
[dPosVector_W, dRot3_WfromTB] = objTargetEmulator.GetPose();

% Assign data for test

dCameraPosition_TB = [1000; 0; 0];
dSunPosition_TB   = [980; 600; 0];

% dDCM_INfromCAM (NOTE: IN is used here for legacy reasons)
strCamera.dQuat_INfromCAM = simulateTBpointing_PosOnly(dCameraPosition_TB, dPosVector_W, true, false, false);
dDCM_INfromCAM = Quat2DCM(strCamera.dQuat_INfromCAM, true);

strShapeModel = orderfields(strShapeModel);
strTargetBodyData.strShapeModel = strShapeModel;
strTargetBodyData.dDCM_INfromTB = dRot3_WfromTB;

strTargetBodyData = orderfields(strTargetBodyData);

% Camera (object and struct)
dFocalLength = 1E3;
ui32OpticalCentre_uv = [512, 512];
ui32ImageSize = [1024, 1024];

objCameraIntrisics = CCameraIntrinsics(dFocalLength, ui32OpticalCentre_uv, ui32ImageSize);  
disp(objCameraIntrisics)

% Define camera object
objCamera = CProjectiveCamera(objCameraIntrisics);

strCameraData.dDCM_INfromCAM = dDCM_INfromCAM;
strCameraData.dPosition_IN = dCameraPosition_TB; % TO MODIFY
strCameraData.dResX = ui32ImageSize(1);
strCameraData.dResY = ui32ImageSize(2);
strCameraData.dKcam = objCameraIntrisics.K;
strCameraData       = orderfields(strCameraData);

% Assign data for CheckLMvisibility_rayTrace_MEX (LEGACY)
dFovHW = objCameraIntrisics.GetFovHW;

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

i_strTargetBody = objTargetEmulator.getTargetStruct();

%% legacyTest_CheckLMvisibility_rayTrace_MEX
% Legacy function for equivalence test (MEX because MATLAB version is too slow)
% MEx equivalence test and timing
% i_strTargetBody = orderfields(i_strTargetBody);
% strCamera = orderfields(strCamera);
% strVisibilityCheckOptions = orderfields(strVisibilityCheckOptions);
% 
% fcnHandle_LEGACY = @() CheckLMvisibility_rayTrace_MEX([double(ui32pointsIDs); dPointsPositionsGT_TB], ...
%     i_strTargetBody, strCamera, dSunPosition_TB./norm(dSunPosition_TB), strVisibilityCheckOptions);
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
[bAllPointsVisibilityMask_legacyEllipsLocalPA, ~] = RayTracePointVisibility_EllipsLocalPA_MEX(uint32(ui32pointsIDs), ...
                                                                                                   dPointsPositionsGT_TB, ...
                                                                                                   strTargetBodyData, ...
                                                                                                   strCameraData, ...
                                                                                                   dSunPosition_TB, ...
                                                                                                   strFcnOptions, ...
                                                                                                   bDEBUG_MODE);

%% test_RayTracePointVisibility_ShadowRays
bTwoSidedTest = false;

[bAllPointsVisibilityMask_RTwithShadowRays, ~] = RayTracePointVisibility_ShadowRays(uint32(ui32pointsIDs), ...
                                                                                    dPointsPositionsGT_TB, ...
                                                                                    strTargetBodyData, ...
                                                                                    strCameraData, ...
                                                                                    dSunPosition_TB, ...
                                                                                    bDEBUG_MODE, ...
                                                                                    bTwoSidedTest); %#codegen



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS

% 3D plot
% Set flag for background color
bUseBlackBackground = true; % Set to false for white background

objFigPointCloud = figure('Renderer','opengl');

% Set background color based on flag
if bUseBlackBackground
    set(gca, 'Color', 'k'); % Axes background
    set(gcf, 'Color', 'k'); % Figure background
    textColor = 'w'; % White text
else
    set(gca, 'Color', 'w'); % White background
    set(gcf, 'Color', 'w');
    textColor = 'k'; % Black text
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
xlabel('X [m]', 'Color', textColor);
ylabel('Y [m]', 'Color', textColor);
zlabel('Z [m]', 'Color', textColor);
set(gca, 'XColor', textColor, 'YColor', textColor, 'ZColor', textColor);

% camproj('perspective'); % Use perspective projection
% campos(dCameraPosition_TB'); % Set camera to camera position
% camtarget(-dCameraPosition_TB')
view(dCameraPosition_TB'); % Camera direction % TODO (PC) use this when showing plots of emulator!

% TODO (img like visualization
% objFigImgLike = figure('Renderer','opengl');
% Project points 

% Plot sun direction (TODO use this representation for the Sun in SLAM plots, much better)
hold on;
lineScale = 1.5; 
objDirToSun = plot3([0, lineScale * dSunPosition_TB(1)], ...
                    [0, lineScale * dSunPosition_TB(2)], ...
                    [0, lineScale * dSunPosition_TB(3)], 'r-', ...
                    'LineWidth', 2, 'DisplayName', 'To Sun');
legend([objPatchModel, objPointCloud_GT, objDirToSun], 'TextColor', textColor);
hold off;

% Show visible points in 3D plot
figure(objFigPointCloud);
hold on;
% Local phase angle check (ellipsoid assumption)
dPointsPositionsGT_RTwithEllipsLocalPA = dPointsPositionsGT_TB(:, bAllPointsVisibilityMask_legacyEllipsLocalPA);
objPointCloud_RTwithEllipsLocalPA = plot3(dPointsPositionsGT_RTwithEllipsLocalPA(1, :), dPointsPositionsGT_RTwithEllipsLocalPA(2, :), ...
                dPointsPositionsGT_RTwithEllipsLocalPA(3,:), 'b.', 'MarkerSize', 5, 'DisplayName', 'RT + Ellips. Local PA');

% TODO (PC): add 2D frame plot with rectangle 
% 2D Image projection plot
[dProjectedPoints_UV]   = pinholeProjectArrayHP_DCM(objCameraIntrisics.K, dDCM_INfromCAM', dCameraPosition_TB, dPointsPositionsGT_TB);
bPointWithinFoV = ((dProjectedPoints_UV(1, :) > 0 & dProjectedPoints_UV(1, :) < objCameraIntrisics.ImageSize(1)) &...
                   (dProjectedPoints_UV(2, :) > 0 & dProjectedPoints_UV(2, :) < objCameraIntrisics.ImageSize(2)))';

% Determine visible and illuminated points
bVisiblePoints = bPointWithinFoV & bAllPointsVisibilityMask_legacyEllipsLocalPA;

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
objVisibleProj = scatter(dProjectedPoints_UV(1, bVisiblePoints), dProjectedPoints_UV(2, bVisiblePoints), 5, 'b', 'filled', 'DisplayName', 'RT + Ellips. Local PA');

% Flip Y-axis (Image coordinates have origin at top-left)
set(gca, 'YDir', 'reverse');
set(gca, 'XColor', 'w', 'YColor', 'w'); % White axis lines

% Detector rectangle (sensor boundaries)
% Draw the detector rectangle using `rectangle()`
objCamDetector = rectangle('Position', [0, 0, objCameraIntrisics.ImageSize(1), objCameraIntrisics.ImageSize(2)], 'EdgeColor', 'r', 'LineWidth', 2);

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

legend([objVisibleProj, objMeshProj], 'TextColor', 'w');
hold off;

