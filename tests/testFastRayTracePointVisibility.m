% Call parent script to get data
testSimulationSetup();
% [dPointsPositionsGT_TB_check, ui32pointsIDs_check] = objTargetEmulator.GetPointsInTargetFrame(ui32pointsIDs, false);
[dPosVector_W, dRot3_WfromTB] = objTargetEmulator.GetPose();

% Assign data for FastRayTracePointVisibility()
ui32pointsIDs = ui32pointsIDs_check;
dPointsPositions_TB = dPointsPositionsGT_TB_check;

dCameraPosition_IN = [1000; 0; 0];
dSunPosition_TB   = [-980; 0; 0];

% dDCM_INfromCAM
strCamera.dQuat_INfromCAM = simulateTBpointing_PosOnly(dCameraPosition_IN, dPosVector_W, true, false, false);
dDCM_INfromCAM = Quat2DCM(strCamera.dQuat_INfromCAM, true);

strShapeModel = orderfields(strShapeModel);
strTargetBodyData.strShapeModel = strShapeModel;
strTargetBodyData.dDCM_INfromTB = dRot3_WfromTB;

strTargetBodyData = orderfields(strTargetBodyData);

% Camera
focalLength = 1E3;
center_uv = [512, 512];
image_size = [1024, 1024];

objCamera = CCameraIntrinsics(focalLength, center_uv, image_size) %#ok<NOPTS>

strCameraData.dDCM_INfromCAM = dDCM_INfromCAM;
strCameraData.dPosition_IN = dCameraPosition_IN; % TO MODIFY
strCameraData.dResX = image_size(1);
strCameraData.dResY = image_size(2);
strCameraData.dKcam = objCamera.K;
strCameraData       = orderfields(strCameraData);

% Assign data for CheckLMvisibility_rayTrace_MEX (LEGACY)
dFovHW = objCamera.GetFovHW;

strCamera.dFovX             = dFovHW(2); % [rad]
strCamera.dFovY             = dFovHW(1); % [rad]
% strCamera.dQuat_INfromCAM   = DCM2quat(dDCM_INfromCAM, true);
strCamera.bIS_JPL_QUAT      = true;
strCamera.dCAMpos_IN        = dCameraPosition_IN;
strCamera = orderfields(strCamera);

% Determines max angle between Sun direction and LM direction
strVisibilityCheckOptions.dIllumAngleThr = deg2rad(65); % [rad] % Lower value, more permissible
% Determines max angle between -los and LM direction in TF. Angle to assign is between +Los and the
strVisibilityCheckOptions.dLosAngleThr = deg2rad(65); % [rad] % Lower value, more permissible
strVisibilityCheckOptions = orderfields(strVisibilityCheckOptions);

strFcnOptions.dIllumAngleThr = strVisibilityCheckOptions.dIllumAngleThr;
strFcnOptions.dLosAngleThr   = strVisibilityCheckOptions.dLosAngleThr;
strFcnOptions.bENABLE_HEUR_GEOM_FEAS_CHECK = true;

i_strTargetBody = objTargetEmulator.getTargetStruct();

% Legacy function for equivalence test
% tic
% [o_dIsLMvisibleMask] = CheckLMvisibility_rayTrace([double(ui32pointsIDs); dPointsPositions_TB], ...
%     i_strTargetBody, strCamera, dSunPosition_TB./norm(dSunPosition_TB), strVisibilityCheckOptions);
% legacy_timing_matlab = toc;

% Test MATLAB function
bDEBUG_MODE = true;
profile on
tic
[bAllPointsVisibilityMask] = FastRayTracePointVisibility(ui32pointsIDs, dPointsPositions_TB, ...
    strTargetBodyData, strCameraData, dSunPosition_TB, strFcnOptions, bDEBUG_MODE);
new_timing_matlab = toc;
prof1 = profile('info');
profile off


% disp('Sum of output differences against LEGACY:')
% bOutputDiffLegacy = bAllPointsVisibilityMask ~= o_dIsLMvisibleMask;
% sum(abs(bOutputDiffLegacy))
return
% MEx equivalence test and timing
i_strTargetBody = orderfields(i_strTargetBody);
strCamera = orderfields(strCamera);
strVisibilityCheckOptions = orderfields(strVisibilityCheckOptions);

fcnHandle_LEGACY = @() CheckLMvisibility_rayTrace_MEX([double(ui32pointsIDs); dPointsPositions_TB], ...
    i_strTargetBody, strCamera, dSunPosition_TB./norm(dSunPosition_TB), strVisibilityCheckOptions);

[dAvgRunTime_LEGACY, dTimings_LEGACY] = AverageFunctionTiming(fcnHandle_LEGACY, 100);

strTargetBodyData = orderfields(strTargetBodyData);
strCameraData = orderfields(strCameraData);
strFcnOptions = orderfields(strFcnOptions);

fcnHandle_mex = @() FastRayTracePointVisibility_MEX(uint32(ui32pointsIDs), dPointsPositions_TB, ...
    strTargetBodyData, strCameraData, dSunPosition_TB, strFcnOptions, bDEBUG_MODE);

[bAllPointsVisibilityMask_MEX] = fcnHandle_mex();

[dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle_mex, 100);

disp('Sum of output differences against MEX:')
bOutputDiff = bAllPointsVisibilityMask ~= bAllPointsVisibilityMask_MEX;
sum(abs(bOutputDiff))



