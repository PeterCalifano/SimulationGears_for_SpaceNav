function [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint] = LaserRangefinderModel( ...
    strTargetModelData, ...
    dBeamDirection_TB, ...
    dSensorOrigin_TB, ...
    dMeasWhiteNoiseSigma, ...
    dConstantBias, ...
    dTargetPosition_TB, ...
    dMeasValidInterval, ...
    bEnableNoiseModels, ...
    bEnableHeuristicPruning, ...
    bEnableValidityChecks) %#codegen
arguments
    strTargetModelData      (1,1) struct    {isstruct, isscalar}
    dBeamDirection_TB       (3,1) double    {isnumeric, isvector}
    dSensorOrigin_TB        (3,1) double    {isnumeric, isvector} % TBC
    dMeasWhiteNoiseSigma    (1,1) double    {isnumeric, isscalar}
    dConstantBias           (1,1) double    {isnumeric, isscalar}
    dTargetPosition_TB      (3,1) double    {isnumeric, isvector} = [0;0;0]
    dMeasValidInterval      (2,1) double    {isnumeric, isvector} = [0; 1e5]
    bEnableNoiseModels      (1,1) logical   {islogical, isscalar} = false
    bEnableHeuristicPruning (1,1) logical   {islogical, isscalar} = false
    bEnableValidityChecks   (1,1) logical   {islogical, isscalar} = false
end

% Check if the input structure contains the required fields
assert(isfield(strTargetModelData, 'i32triangVertexPtrs') || ...
    isfield(strTargetModelData, 'dVerticesPositions'), ...
    'Error: strTargetModelData must contain fields i32triangVertexPtrs and dVerticesPositions');

% Costly checks (enable flag for this?)
assert( (abs(norm(dBeamDirection_TB) - 1) < 1.5*eps), 'Error: dBeamDirection_TB must be a unit vector');

% Perform preliminary pointing check using dot-product
dRayOriginToTargetOrigin = dTargetPosition_TB - dSensorOrigin_TB;
bPointingCheckPassed = dot(dRayOriginToTargetOrigin, dBeamDirection_TB) > 0; % DEVNOTE: 0 should safe for any realistic scenario

% RayTraceLaserBeam

if bPointingCheckPassed

    bInsersectionFlag = false;
    dMeasDistance = -ones(1,1);
    dIntersectionPoint = -ones(3,1);

    [bInsersectionFlag(:), dMeasDistance(:), dIntersectionPoint(:)] = RayTraceLaserBeam(strTargetModelData, ...
        dBeamDirection_TB, ...
        dSensorOrigin_TB, ...
        dTargetPosition_TB, ...
        bEnableHeuristicPruning);

else
    dMeasDistance       = -1.0;
    bInsersectionFlag   = false;
    bValidityFlag       = false;
    dIntersectionPoint  = zeros(3,1);
    return
end

% Noise models application
% TODO (PC): implement noise models (have a look at Hera's model)
bValidityFlag = true;

if bEnableNoiseModels == true
    dMeasErr = LaserRangefinderNoiseModel(dMeasWhiteNoiseSigma, dMeasDistance, dConstantBias);
else
    dMeasErr = zeros(1,1);
end

% Add measurement error
dMeasDistance(:) = dMeasDistance(:) + dMeasErr;

% Validity checks
if bEnableValidityChecks == true
    if (dMeasDistance < dMeasValidInterval(1) && dMeasDistance > dMeasValidInterval(2)) || dMeasDistance < 0
        bValidityFlag = false;
    end
end

end
