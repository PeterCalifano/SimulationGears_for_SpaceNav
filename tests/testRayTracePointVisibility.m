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

objCamera = CCameraIntrinsics(dFocalLength, ui32OpticalCentre_uv, ui32ImageSize);  
disp(objCamera)

strCameraData.dDCM_INfromCAM = dDCM_INfromCAM;
strCameraData.dPosition_IN = dCameraPosition_TB; % TO MODIFY
strCameraData.dResX = ui32ImageSize(1);
strCameraData.dResY = ui32ImageSize(2);
strCameraData.dKcam = objCamera.K;
strCameraData       = orderfields(strCameraData);

% Assign data for CheckLMvisibility_rayTrace_MEX (LEGACY)
dFovHW = objCamera.GetFovHW;

strCamera.dFovX             = dFovHW(2); % [rad]
strCamera.dFovY             = dFovHW(1); % [rad]
% strCamera.dQuat_INfromCAM   = DCM2quat(dDCM_INfromCAM, true);
strCamera.bIS_JPL_QUAT      = true;
strCamera.dCAMpos_IN        = dCameraPosition_TB;
strCamera = orderfields(strCamera);

% PLOT 
% TODO (PC): add 2D frame plot with rectangle 
objFigPointCloud = figure('Renderer','opengl');

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
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
% camproj('perspective'); % Use perspective projection
% campos(dCameraPosition_TB'); % Set camera to camera position
% camtarget(-dCameraPosition_TB')
% view(dCameraPosition_TB'); % Camera direction % TODO (PC) use this when showing plots of emulator!

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
legend([objPatchModel, objPointCloud_GT, objDirToSun]);
hold off;

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

fcnHandle_mex = @() RayTracePointVisibilityLocalPA_MEX(uint32(ui32pointsIDs), dPointsPositionsGT_TB, ...
    strTargetBodyData, strCameraData, dSunPosition_TB, strFcnOptions, bDEBUG_MODE);

[dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle_mex, 1);

% Get output for visualization
[bAllPointsVisibilityMask_legacyLocalPA, dProjectedPoints_UV] = RayTracePointVisibility_EllipsLocalPA_MEX(uint32(ui32pointsIDs), ...
                                                                                                   dPointsPositionsGT_TB, ...
                                                                                                   strTargetBodyData, ...
                                                                                                   strCameraData, ...
                                                                                                   dSunPosition_TB, ...
                                                                                                   strFcnOptions, ...
                                                                                                   bDEBUG_MODE);

% Show points in 3D plot
figure(objFigPointCloud);
hold on;
dPointsPositionsGT_RTwithLocalPA = dPointsPositionsGT_TB(:, bAllPointsVisibilityMask_legacyLocalPA);
objPointCloud_RTwithLocalPA = plot3(dPointsPositionsGT_RTwithLocalPA(1, :), dPointsPositionsGT_RTwithLocalPA(2, :), ...
                dPointsPositionsGT_RTwithLocalPA(3,:), 'b.', 'MarkerSize', 5, 'DisplayName', 'RT + Local PA');

% TODO: plot points in image plane



%% 




