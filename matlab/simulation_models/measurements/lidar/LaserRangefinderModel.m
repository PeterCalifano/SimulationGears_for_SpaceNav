function [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint, dMeasErr] = ...
    LaserRangefinderModel(strTargetModelData, dBeamDirection_TB, dSensorOrigin_TB, ...
                         dMeasWhiteNoiseSigma, dConstantBias, dTargetPosition_TB, ...
                         dMeasValidInterval, bEnableNoiseModels, bEnableHeuristicPruning, ...
                         bEnableValidityChecks) %#codegen
%% SIGNATURE
% [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint, dMeasErr] = ...
%     LaserRangefinderModel(strTargetModelData, dBeamDirection_TB, dSensorOrigin_TB, ...
%                          dMeasWhiteNoiseSigma, dConstantBias, dTargetPosition_TB, ...
%                          dMeasValidInterval, bEnableNoiseModels, bEnableHeuristicPruning, ...
%                          bEnableValidityChecks)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Simulate a range return from the nearest forward mesh intersection.
% Accept either prepared fixed-size ray data or the legacy vertices/indices.
% Apply configured white noise and bias only after a geometric hit. Return an
% invalid measurement on a miss, even when range-validity checks are disabled.
% Keep sensor and geometry inputs in the same length unit.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strTargetModelData      Prepared strRayData, or raw vertices and triangle indices.
% dBeamDirection_TB       (3,1) Unit beam direction in the target frame [-].
% dSensorOrigin_TB        (3,1) Sensor origin in the target frame [length].
% dMeasWhiteNoiseSigma    White-noise standard deviation [length].
% dConstantBias           Additive sensor bias [length].
% dTargetPosition_TB      Legacy compatibility argument; default zero.
% dMeasValidInterval      (2,1) Inclusive lower/upper range bounds [length].
% bEnableNoiseModels      Apply configured noise and bias; default false.
% bEnableHeuristicPruning Legacy compatibility flag; default false.
% bEnableValidityChecks   Enforce range bounds; default false.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMeasDistance       Range including enabled sensor error [length]; -1 on a miss.
% bInsersectionFlag   Geometric hit flag; retain the existing public spelling.
% bValidityFlag       Measurement satisfies geometry and enabled range checks.
% dIntersectionPoint  (3,1) Geometric target-frame hit [length]; zero on a miss.
% dMeasErr            Realized additive sensor error [length].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 16-01-2025    Pietro Califano     First implementation for RCS-1 simulator
% 21-01-2026    Pietro Califano     Review and optimization for codegen
% 24-08-2026    Pietro Califano, Codex gpt-5.6     Reject ranges outside either validity bound.
% 05-10-2026    Pietro Califano, Codex (GPT-6)  Trace the closest forward surface without centre clipping.
% 08-10-2026    Pietro Califano, Codex (GPT-6)  Share sensor policy with direct-array MEX entry points.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RayTraceTriangMesh, ApplyLidarMeasurementError.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strTargetModelData      (1, 1) struct
    dBeamDirection_TB       (3, 1) double
    dSensorOrigin_TB        (3, 1) double
    dMeasWhiteNoiseSigma    (1, 1) double
    dConstantBias           (1, 1) double
    dTargetPosition_TB      (3, 1) double = [0; 0; 0]
    dMeasValidInterval      (2, 1) double = [0; 1e5]
    bEnableNoiseModels      (1, 1) logical = false
    bEnableHeuristicPruning (1, 1) logical = false
    bEnableValidityChecks   (1, 1) logical = false
end

arguments (Output)
    dMeasDistance      (1, 1) double
    bInsersectionFlag  (1, 1) logical
    bValidityFlag      (1, 1) logical
    dIntersectionPoint (3, 1) double
    dMeasErr           (1, 1) double
end

%% Function code

% Require one complete geometry representation before tracing.
assert(isfield(strTargetModelData, 'strRayData') || ...
    (isfield(strTargetModelData, 'i32triangVertexPtrs') && ...
    isfield(strTargetModelData, 'dVerticesPositions')), ...
    'LaserRangefinderModel:MissingMesh', ...
    'Supply prepared strRayData or both raw vertices and triangle indices.');

if coder.target('MATLAB')
    % Validate the unit beam at the source boundary; preparation owns mesh checks.
    assert(abs(norm(dBeamDirection_TB) - 1) < 0.1 * eps('single'), ...
        'LaserRangefinderModel:BeamDirection', 'dBeamDirection_TB must be a unit vector.');
end

% Trace the nearest forward surface regardless of the mesh centre's projection.
% A ray can hit a protrusion even when the centre is beside or behind its origin.
[bInsersectionFlag, dMeasDistance, dIntersectionPoint] = RayTraceTriangMesh( ...
    strTargetModelData, dBeamDirection_TB, dSensorOrigin_TB, ...
    dTargetPosition_TB, bEnableHeuristicPruning);

% Share sensor policy with the direct-array codegen entry point.
[dMeasDistance, bValidityFlag, dMeasErr] = ...
    ApplyLidarMeasurementError(dMeasDistance, bInsersectionFlag, dMeasWhiteNoiseSigma, ...
                              dConstantBias, dMeasValidInterval, ...
                              bEnableNoiseModels, bEnableValidityChecks);

end
