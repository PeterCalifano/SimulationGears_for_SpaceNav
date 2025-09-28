

% Test setup
% Call parent script to get data
testSimulationSetup();
% [dPointsPositionsGT_TB_check, ui32pointsIDs_check] = objTargetEmulator.GetPointsInTargetFrame(ui32pointsIDs, false);
[dPosVector_W, dRot3_WfromTB] = objTargetEmulator.GetPose();

strShapeModel = orderfields(strShapeModel);

% strTargetModelData      (1,1) struct    {isstruct, isscalar}
% dBeamDirection_TB       (3,1) double    {isnumeric, isvector}
% dSensorOrigin_TB        (3,1) double    {isnumeric, isvector} % TBC
% dMeasWhiteNoiseSigma    (1,1) double    {isnumeric, isscalar}
% dConstantBias           (1,1) double    {isnumeric, isscalar}
% dTargetPosition_TB      (3,1) double    {isnumeric, isvector} = [0;0;0]
% dMeasValidInterval      (2,1) double    {isnumeric, isvector} = [0; 1e5]
% bEnableNoiseModels      (1,1) logical   {islogical, isscalar} = false
% bEnableHeuristicPruning (1,1) logical   {islogical, isscalar} = false
% bEnableValidityChecks   (1,1) logical   {islogical, isscalar} = false

strTargetModelData.i32triangVertexPtrs  = int32(strShapeModel.ui32triangVertexPtr);
strTargetModelData.dVerticesPositions   = strShapeModel.dVerticesPos;

dMeasWhiteNoiseSigma = 10; % [m]
dConstantBias = 0;
dMeasValidInterval = [0; 2e3];
bEnableNoiseModels = false;
bEnableHeuristicPruning = false;
bEnableValidityChecks = true;
dTargetPosition_TB = dPosVector_W;

%% LaserRangefinderModel_MATLAB_0
% Description Intersection exists and is valid, no noise model
dSensorOrigin_TB    = [-300; 0; 0] - dTargetPosition_TB; % [m]
dBeamDirection_TB   = - dSensorOrigin_TB./norm(dSensorOrigin_TB);

[dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint] = LaserRangefinderModel( ...
    strTargetModelData, ...
    dBeamDirection_TB, ...
    dSensorOrigin_TB, ...
    dMeasWhiteNoiseSigma, ...
    dConstantBias, ...
    dTargetPosition_TB, ...
    dMeasValidInterval, ...
    bEnableNoiseModels, ...
    bEnableHeuristicPruning, ...
    bEnableValidityChecks);

assert( dMeasDistance > 0)
assert( bInsersectionFlag == true )
assert( bValidityFlag == true ) 
assert( dMeasDistance < norm(dSensorOrigin_TB));
assert( abs(dMeasDistance - norm(dIntersectionPoint - dSensorOrigin_TB)) < eps('single'))

%% LaserRangefinderModel_MATLAB_1
% Description Intersection does not exist 
dSensorOrigin_TB    = [-300; 0; 0]; % [m]
dBeamDirection_TB   = dSensorOrigin_TB./norm(dSensorOrigin_TB); % Looks in the opposite direction

[dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint] = LaserRangefinderModel( ...
    strTargetModelData, ...
    dBeamDirection_TB, ...
    dSensorOrigin_TB, ...
    dMeasWhiteNoiseSigma, ...
    dConstantBias, ...
    dTargetPosition_TB, ...
    dMeasValidInterval, ...
    bEnableNoiseModels, ...
    bEnableHeuristicPruning, ...
    bEnableValidityChecks);

assert( dMeasDistance < 0)
assert( bInsersectionFlag == false)
assert( bValidityFlag == false) 
assert( all(dIntersectionPoint == 0) ) 


%% LaserRangefinderModel_MATLAB_2
% Description Intersection exists and is valid, with noise model
dSensorOrigin_TB    = [-300; 0; 0]; % [m]
dBeamDirection_TB   = -dSensorOrigin_TB./norm(dSensorOrigin_TB); % Looks in the opposite direction
bEnableNoiseModels  = true;

[dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint] = LaserRangefinderModel( ...
    strTargetModelData, ...
    dBeamDirection_TB, ...
    dSensorOrigin_TB, ...
    dMeasWhiteNoiseSigma, ...
    dConstantBias, ...
    dTargetPosition_TB, ...
    dMeasValidInterval, ...
    bEnableNoiseModels, ...
    bEnableHeuristicPruning, ...
    bEnableValidityChecks);

assert( dMeasDistance > 0)
assert( bInsersectionFlag == true)
assert( bValidityFlag == true) 
assert( abs(dMeasDistance - norm(dIntersectionPoint - dSensorOrigin_TB)) < 6 * dMeasWhiteNoiseSigma)
return

% % MEx equivalence test and timing
fcnHandle_mex = @() FastRayTracePointVisibility_MEX(uint32(ui32pointsIDs), dPointsPositions_TB, ...
    strTargetBodyData, strCameraData, dSunPosition_TB, strFcnOptions, bDEBUG_MODE);

[dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle_mex, 100);

disp('Sum of output differences against MEX:')


