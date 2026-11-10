function [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint, dMeasErr] = ...
    LaserRangefinderModelArrays(ui32TriangleCount, dVertex0, dEdge1, dEdge2, ...
                               dNodeMin, dNodeMax, ui32NodeLeft, ui32NodeRight, ...
                               ui32LeafStart, ui32LeafCount, ui32TriangleOrder, bUseBvh, ...
                               dBeamDirection, dSensorOrigin, dWhiteNoiseSigma, ...
                               dConstantBias, dValidInterval, bEnableNoise, bEnableChecks) %#codegen
%% SIGNATURE
% [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint, dMeasErr] = ...
%     LaserRangefinderModelArrays(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the complete prepared LiDAR model through direct numeric inputs.
% Share tracing and sensor policy with LaserRangefinderModel. Codegen uses this
% entry to remove struct-field marshalling at the MEX boundary. The generated MATLAB
% facade retains the public model's existing ten-input calling convention.
% Prepare and validate geometry once before repeated unit-direction queries.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32TriangleCount       Active source triangle count.
% dVertex0/dEdge1/dEdge2   (3,N) prepared triangle origins and edges.
% dNodeMin/dNodeMax       (3,M) conservative node bounds.
% ui32NodeLeft/Right      (1,M) child indices.
% ui32LeafStart/Count     (1,M) leaf spans in the source permutation.
% ui32TriangleOrder       (1,N) original source IDs in traversal order.
% bUseBvh                 Select BVH traversal; false selects the flat scan.
% dBeamDirection          (3,1) unit beam direction in the mesh frame.
% dSensorOrigin           (3,1) ray origin in that frame and length unit.
% dWhiteNoiseSigma        Sensor white-noise standard deviation.
% dConstantBias           Additive sensor bias.
% dValidInterval          (2,1) inclusive measured-range bounds.
% bEnableNoise/Checks     Enable the existing sensor-error and validity policies.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMeasDistance           Measured range; -1 on a geometric miss.
% bInsersectionFlag       Geometric hit flag; preserve the public spelling.
% bValidityFlag           Measurement passes geometry and enabled range checks.
% dIntersectionPoint      (3,1) geometric hit point; zero on a miss.
% dMeasErr                Realized additive sensor error; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Avoid struct-field copies in complete LiDAR MEX calls.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% TraceTriangleRayArrays, ApplyLidarMeasurementError.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    ui32TriangleCount  (1, 1) uint32
    dVertex0           (3, :) double
    dEdge1             (3, :) double
    dEdge2             (3, :) double
    dNodeMin           (3, :) double
    dNodeMax           (3, :) double
    ui32NodeLeft       (1, :) uint32
    ui32NodeRight      (1, :) uint32
    ui32LeafStart      (1, :) uint32
    ui32LeafCount      (1, :) uint32
    ui32TriangleOrder  (1, :) uint32
    bUseBvh            (1, 1) logical
    dBeamDirection     (3, 1) double
    dSensorOrigin      (3, 1) double
    dWhiteNoiseSigma   (1, 1) double
    dConstantBias      (1, 1) double
    dValidInterval     (2, 1) double
    bEnableNoise       (1, 1) logical
    bEnableChecks      (1, 1) logical
end

arguments (Output)
    dMeasDistance      (1, 1) double
    bInsersectionFlag  (1, 1) logical
    bValidityFlag      (1, 1) logical
    dIntersectionPoint (3, 1) double
    dMeasErr           (1, 1) double
end

% TODO (PC): Reduce generated MEX input-protection copies of validated geometry.
% Keep the verified source/MEX semantics and cached geometry ownership unchanged.

% Use the same nearest-forward interval as the public prepared mesh tracer.
strQuery = struct('dDirection', dBeamDirection, 'bAnyHit', false, ...
                  'bTwoSided', true, 'dMinDistance', eps, 'dMaxDistance', Inf, ...
                  'ui32IgnoreTriangle', uint32(0));

[bInsersectionFlag, dTrueDistance, dIntersectionPoint] = ...
    TraceTriangleRayArrays(ui32TriangleCount, dVertex0, dEdge1, dEdge2, ...
                           dNodeMin, dNodeMax, ui32NodeLeft, ui32NodeRight, ...
                           ui32LeafStart, ui32LeafCount, ui32TriangleOrder, ...
                           bUseBvh, dSensorOrigin, strQuery);

% Apply the shared sensor policy without duplicating noise or range decisions.
[dMeasDistance, bValidityFlag, dMeasErr] = ...
    ApplyLidarMeasurementError(dTrueDistance, bInsersectionFlag, dWhiteNoiseSigma, ...
                              dConstantBias, dValidInterval, bEnableNoise, bEnableChecks);
end
