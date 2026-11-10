function [bIntersectionFlag, dUbarycenCoord, dVbarycenCoord, ...
    dtRangeToIntersection, dIntersectionPoint] = RayTriangleIntersection_MollerTrumbore( ...
    dRayOrigin, dRayDirection, dTriangVert0, dTriangVert1, dTriangVert2, bTwoSidedTest) %#codegen
%% SIGNATURE
% [bHit, dU, dV, dRange, dPoint] = RayTriangleIntersection_MollerTrumbore(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Intersect a forward ray using shared Moller-Trumbore mathematics.
% Algorithm reference: Moller and Trumbore, 1997, doi:10.1145/1198555.1198746.
% Preserve the established absolute determinant tolerance and sidedness.
% Construct the point only when requested. Non-unit directions return a ray
% parameter rather than a physical range.
% Example: [bHit,~,~,dRange] = RayTriangleIntersection_MollerTrumbore( ...
%     [0.2;0.2;1],[0;0;-1],[0;0;0],[1;0;0],[0;1;0],true);
% Output: true and 1.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRayOrigin/dRayDirection  (3,1) ray in the triangle frame.
% dTriangVert0/1/2         (3,1) triangle vertices [length].
% bTwoSidedTest            Accept either winding; default true.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bIntersectionFlag        Forward hit; false on a miss.
% dUbarycenCoord/dVbarycenCoord  Barycentric coordinates; zero on a miss.
% dtRangeToIntersection    Ray parameter; zero on a miss.
% dIntersectionPoint       Hit position; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-02-2025  Pietro Califano        Implement the original paper with shadow rays.
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Reuse the shared barycentric kernel.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% IntersectTriangleEdges (private).
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dRayOrigin (3, 1) double
    dRayDirection (3, 1) double
    dTriangVert0 (3, 1) double
    dTriangVert1 (3, 1) double
    dTriangVert2 (3, 1) double
    bTwoSidedTest (1, 1) logical = true
end

arguments (Output)
    bIntersectionFlag (1, 1) logical
    dUbarycenCoord (1, 1) double
    dVbarycenCoord (1, 1) double
    dtRangeToIntersection (1, 1) double
    dIntersectionPoint (3, 1) double
end

% Reject parallel or culled faces before the shared barycentric test.
dEdge1 = dTriangVert1 - dTriangVert0;
dEdge2 = dTriangVert2 - dTriangVert0;
dCrossEdge2 = cross(dRayDirection, dEdge2);
dDet = dot(dEdge1, dCrossEdge2);
dInverseDet = 0;

if (bTwoSidedTest && abs(dDet) >= eps) || (~bTwoSidedTest && dDet >= eps)
    dInverseDet = 1 / dDet;
end

[bIntersectionFlag, dtRangeToIntersection, dUbarycenCoord, dVbarycenCoord] = ...
    IntersectTriangleEdges(dRayOrigin - dTriangVert0, dRayDirection, ...
    dEdge1, dEdge2, dCrossEdge2, dInverseDet, eps, Inf);

% Preserve barycentric reconstruction for the established public API.
dIntersectionPoint = zeros(3, 1);

if nargout >= 5 && bIntersectionFlag
    dIntersectionPoint = (1 - dUbarycenCoord - dVbarycenCoord) * dTriangVert0 + ...
        dUbarycenCoord * dTriangVert1 + dVbarycenCoord * dTriangVert2;
end
end
