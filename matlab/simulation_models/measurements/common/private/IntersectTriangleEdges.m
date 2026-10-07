function [bHit, dDistance, dU, dV] = IntersectTriangleEdges(dOriginFromVertex, ...
    dDirection, dEdge1, dEdge2, dCrossEdge2, dInverseDet, dMinDistance, dMaxDistance) %#codegen
%% SIGNATURE
% [bHit, dDistance, dU, dV] = IntersectTriangleEdges(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate shared Moller-Trumbore barycentric and interval tests.
% Receive prepared edges and direction coefficients. Use a zero inverse
% determinant for rejected orientations. Retain inclusive triangle edges and an
% open lower/closed upper ray-parameter interval.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dOriginFromVertex  (3,1) origin minus triangle vertex zero [length].
% dDirection         (3,1) supplied ray direction.
% dEdge1/dEdge2      (3,1) triangle edges [length].
% dCrossEdge2        (3,1) cross(direction, edge2).
% dInverseDet        Reciprocal determinant; zero rejects the orientation.
% dMinDistance       Open lower ray-parameter limit.
% dMaxDistance       Closed upper ray-parameter limit.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bHit              Intersection within the supplied interval.
% dDistance/dU/dV    Ray parameter and barycentric coordinates; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dOriginFromVertex (3, 1) double
    dDirection (3, 1) double
    dEdge1 (3, 1) double
    dEdge2 (3, 1) double
    dCrossEdge2 (3, 1) double
    dInverseDet (1, 1) double
    dMinDistance (1, 1) double
    dMaxDistance (1, 1) double
end

arguments (Output)
    bHit (1, 1) logical
    dDistance (1, 1) double
    dU (1, 1) double
    dV (1, 1) double
end

coder.inline('always');
bHit = false;
dDistance = 0;
dU = 0;
dV = 0;
if dInverseDet == 0
    return
end

% Reject the first coordinate before computing the second cross product.
dCandidateU = dot(dOriginFromVertex, dCrossEdge2) * dInverseDet;
if dCandidateU < 0 || dCandidateU > 1
    return
end

dCrossEdge1 = cross(dOriginFromVertex, dEdge1);
dCandidateV = dot(dDirection, dCrossEdge1) * dInverseDet;
if dCandidateV < 0 || dCandidateU + dCandidateV > 1
    return
end

% Enforce the caller's interval without constructing an intersection point.
dCandidateDistance = dot(dEdge2, dCrossEdge1) * dInverseDet;
if dCandidateDistance <= dMinDistance || dCandidateDistance > dMaxDistance
    return
end

bHit = true;
dDistance = dCandidateDistance;
dU = dCandidateU;
dV = dCandidateV;

end
