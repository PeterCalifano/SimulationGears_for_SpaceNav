function [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint, dMeasErr] = LaserRangefinderModel( ...
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
arguments (Input)
    strTargetModelData      (1,1) struct 
    dBeamDirection_TB       (3,1) double 
    dSensorOrigin_TB        (3,1) double  
    dMeasWhiteNoiseSigma    (1,1) double 
    dConstantBias           (1,1) double 
    dTargetPosition_TB      (3,1) double  = [0;0;0]
    dMeasValidInterval      (2,1) double  = [0; 1e5]
    bEnableNoiseModels      (1,1) logical = false
    bEnableHeuristicPruning (1,1) logical = false
    bEnableValidityChecks   (1,1) logical = false
end
arguments (Output)
    dMeasDistance       (1,1) double 
    bInsersectionFlag   (1,1) logical
    bValidityFlag       (1,1) logical
    dIntersectionPoint  (3,1) double 
    dMeasErr            (1,1) double 
end
%% SIGNATURE
% [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint] = LaserRangefinderModel(strTargetModelData, ...
%                                                                                               dBeamDirection_TB, ...
%                                                                                               dSensorOrigin_TB, ...
%                                                                                               dMeasWhiteNoiseSigma, ...
%                                                                                               dConstantBias, ...
%                                                                                               dTargetPosition_TB, ...
%                                                                                               dMeasValidInterval, ...
%                                                                                               bEnableNoiseModels, ...
%                                                                                               bEnableHeuristicPruning, ...
%                                                                                               bEnableValidityChecks) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function implementing sensor model for a Lidar range finder using ray tracing. Measurement error is
% modelled as white noise since errors in shape and pointing are inherently in the ray tracing approach.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strTargetModelData      (1,1) struct  
% dBeamDirection_TB       (3,1) double  
% dSensorOrigin_TB        (3,1) double  % TBC
% dMeasWhiteNoiseSigma    (1,1) double  
% dConstantBias           (1,1) double  
% dTargetPosition_TB      (3,1) double  = [0;0;0]
% dMeasValidInterval      (2,1) double  = [0; 1e5]
% bEnableNoiseModels      (1,1) logical = false
% bEnableHeuristicPruning (1,1) logical = false
% bEnableValidityChecks   (1,1) logical = false
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMeasDistance       (1,1) double
% bInsersectionFlag   (1,1) logical
% bValidityFlag       (1,1) logical
% dIntersectionPoint  (3,1) double
% dMeasErr            (1,1) double
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 16-01-2025    Pietro Califano     First implementation for RCS-1 simulator
% 21-01-2026    Pietro Califano     Review and optimization for codegen
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Check if the input structure contains the required fields
assert(isfield(strTargetModelData, 'i32triangVertexPtrs') || ...
    isfield(strTargetModelData, 'dVerticesPositions'), ...
    'Error: strTargetModelData must contain fields i32triangVertexPtrs and dVerticesPositions');

if coder.target('MATLAB')
    % Costly checks (enable flag for this?)
    assert( (abs(norm(dBeamDirection_TB) - 1) < 0.1*eps('single')), 'Error: dBeamDirection_TB must be a unit vector');
end

% Perform preliminary pointing check using dot-product
dRayOriginToTargetOrigin = dTargetPosition_TB - dSensorOrigin_TB;
bPointingCheckPassed = dot(dRayOriginToTargetOrigin, dBeamDirection_TB) > 0; % DEVNOTE: 0 should safe for any realistic scenario

% RayTraceLaserBeam
dMeasDistance       = -1.0;
bInsersectionFlag   = false;
bValidityFlag       = false;
dIntersectionPoint  = zeros(3,1);
dMeasErr = 0.0;

if bPointingCheckPassed

    bInsersectionFlag = false;
    dMeasDistance = -ones(1,1);
    dIntersectionPoint = -ones(3,1);

    [bInsersectionFlag(:), dMeasDistance(:), dIntersectionPoint(:)] = RayTraceTriangMesh(strTargetModelData, ...
                                                                                    dBeamDirection_TB, ...
                                                                                    dSensorOrigin_TB, ...
                                                                                    dTargetPosition_TB, ...
                                                                                    bEnableHeuristicPruning);

else
    return
end

% Noise models application
% TODO (PC): implement noise models (have a look at Hera's model)
bValidityFlag = true;

if bEnableNoiseModels == true
    dMeasErr(:) = LaserRangefinderNoiseModel(dMeasWhiteNoiseSigma, dMeasDistance, dConstantBias);
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
