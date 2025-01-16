function [dMeasDistance, bInsersectionFlag, bValidityFlag] = LaserRangefinderModel( ...
    strTargetModelData, ...
    dBeamDirection_TB, ...
    dSensorOrigin_TB, ...
    dMeasValidInterval, ...
    bEnableNoiseModels, ...
    bEnableHeuristicPruning, ...
    bEnableValidityChecks) %#codegen
arguments
    strTargetModelData      (1,1) struct {isstruct, isscalar}
    dBeamDirection_TB       (3,1) double {isnumeric,isvector}
    dSensorOrigin_TB        (3,1) double {isnumeric,isvector} % TBC
    dMeasValidInterval      (2,1) double {isnumeric,isvector}   = [0; 1e5]
    bEnableNoiseModels      (1,1) logical {islogical, isscalar} = false
    bEnableHeuristicPruning (1,1) logical {islogical, isscalar} = false
    bEnableValidityChecks   (1,1) logical {islogical, isscalar} = false
end

% Check if the input structure contains the required fields
assert(~isfield(strTargetModelData, 'i32triangVertexPtrs') || ~isfield(strTargetModelData, 'dVerticesPositions'), 'Error: strTargetModelData must contain fields i32VerticesIndex and dVerticesPositions');

% Costly checks (enable flag for this?)
assert( (abs(norm(dBeamDirection_TB) - 1) < 1.5*eps), 'Error: dBeamDirection_TB must be a unit vector');

% RayTraceLaserBeam
bInsersectionFlag = false;
dMeasDistance = -ones(1,1);
dIntersectionPoint_GT = -ones(3,1);

[bInsersectionFlag(:), dMeasDistance(:), dIntersectionPoint_GT(:)] = RayTraceLaserBeam(strTargetModelData, dBeamDirection_TB, bEnableHeuristicPruning);

% Noise models application
% TODO (PC): implement noise models (have a look at Hera's model)
bValidityFlag = true;

if bEnableNoiseModels == true
    dMeasErr = LaserRangefinderNoiseModel(dMeasDistance);
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
