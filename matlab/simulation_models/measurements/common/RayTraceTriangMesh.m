function [bInsersectionFlag, dtParamDistance, dIntersectionPoint] = RayTraceTriangMesh( ...
    strTargetModelData, dRayDirection_TB, dRayOrigin_TB, ...
    dTargetPosition_TB, bEnableHeuristicPruning) %#codegen
%% SIGNATURE
% [bHit, dRange, dPoint] = RayTraceTriangMesh(strTargetModelData, dDirection, dOrigin, ...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Return the nearest forward intersection independently of triangle order.
% Use optional prepared strRayData for exact flat/BVH traversal. Retain legacy
% arguments and miss outputs; do not clip surfaces at the mesh-centre projection.
% Example: [bHit,dRange] = RayTraceTriangMesh(strMesh,[0;0;-1],[0.2;0.2;1]);
% Output: true and 1 for a triangle containing [0.2;0.2;0].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strTargetModelData  dVerticesPositions and one-based i32triangVertexPtrs;
%                     optional strRayData from BuildTriangleRayData.
% dRayDirection_TB    (3,1) direction; use a unit vector for length outputs.
% dRayOrigin_TB       (3,1) origin in the target frame [length].
% dTargetPosition_TB  Compatibility argument; no geometric clipping.
% bEnableHeuristicPruning  Compatibility flag; prepared tracing remains exact.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bInsersectionFlag   Nearest forward hit exists.
% dtParamDistance     Ray parameter, or -1 on a miss.
% dIntersectionPoint  (3,1) hit position, or zeros on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 16-01-2024  Pietro Califano        Implement simulator mesh tracing.
% 16-04-2024  Pietro Califano        Upgrade the intersection primitive.
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Trace the nearest forward hit using prepared geometry.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% TraceTriangleRay, RayTriangleIntersection_MollerTrumbore.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strTargetModelData (1, 1) struct
    dRayDirection_TB (3, 1) double
    dRayOrigin_TB (3, 1) double = zeros(3, 1)
    dTargetPosition_TB (3, 1) double = zeros(3, 1)
    bEnableHeuristicPruning (1, 1) logical = false
end

arguments (Output)
    bInsersectionFlag (1, 1) logical
    dtParamDistance (1, 1) double
    dIntersectionPoint (3, 1) double
end

assert(isfield(strTargetModelData, 'strRayData') || ...
    (isfield(strTargetModelData, 'i32triangVertexPtrs') && ...
    isfield(strTargetModelData, 'dVerticesPositions')), ...
    'RayTraceTriangMesh:MissingMesh', 'Supply both vertices and triangle indices.');

if isfield(strTargetModelData, 'strRayData')
    % Reuse body-frame geometry through the existing mesh argument.
    strQuery = struct('dDirection', dRayDirection_TB, 'bAnyHit', false, ...
        'bTwoSided', true, 'dMinDistance', eps, 'dMaxDistance', Inf, ...
        'ui32IgnoreTriangle', uint32(0));
    [bInsersectionFlag, dtParamDistance, dIntersectionPoint] = ...
        TraceTriangleRay(strTargetModelData.strRayData, dRayOrigin_TB, strQuery);
    return
end

% Keep legacy inputs usable without rebuilding a hierarchy per ray.
bInsersectionFlag = false;
dtParamDistance = -1;
dIntersectionPoint = zeros(3, 1);

for ui32Triangle = uint32(1):uint32(size(strTargetModelData.i32triangVertexPtrs, 2))

    i32Vertices = strTargetModelData.i32triangVertexPtrs(:, ui32Triangle);
    dVertices = strTargetModelData.dVerticesPositions(:, i32Vertices);

    [bCandidate, ~, ~, dDistance] = RayTriangleIntersection_MollerTrumbore( ...
        dRayOrigin_TB, dRayDirection_TB, dVertices(:, 1), dVertices(:, 2), dVertices(:, 3), true);

    if bCandidate && (~bInsersectionFlag || dDistance < dtParamDistance)
        bInsersectionFlag = true;
        dtParamDistance = dDistance;
    end
end

if bInsersectionFlag && nargout >= 3
    dIntersectionPoint = dRayOrigin_TB + dtParamDistance * dRayDirection_TB;
end
end
