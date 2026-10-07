function [bHit, dU, dV, dRange, dPoint] = RayTwoSidedTriangleIntersection_MollerTrembore( ...
    dOrigin, dDirection, dVertex0, dVertex1, dVertex2) %#codegen
%% SIGNATURE
% [bHit,dU,dV,dRange,dPoint] = RayTwoSidedTriangleIntersection_MollerTrembore(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Intersect either winding using the shared barycentric kernel.
% Preserve the legacy 2*eps determinant threshold and signed ray parameter;
% callers decide whether a hit is forward. Retain the original implementation
% credit: Jesus Mena, David Berman and Pietro Califano (13 April 2024).
% Algorithm reference: Moller and Trumbore, 1997, doi:10.1145/1198555.1198746.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dOrigin/dDirection (3,1) ray origin and supplied direction.
% dVertex0/1/2       (3,1) triangle vertices in the same length unit.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bHit              Hit in the complete infinite-line interval.
% dU/dV/dRange      Barycentric coordinates and signed ray parameter; zero on miss.
% dPoint            Barycentric hit position; zero on miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-04-2024  Pietro Califano        Optimize the original two-sided implementation.
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Reuse the shared barycentric kernel.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% IntersectTriangleEdges (private).
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dOrigin (3, 1) double
    dDirection (3, 1) double
    dVertex0 (3, 1) double
    dVertex1 (3, 1) double
    dVertex2 (3, 1) double
end

arguments (Output)
    bHit (1, 1) logical
    dU (1, 1) double
    dV (1, 1) double
    dRange (1, 1) double
    dPoint (3, 1) double
end

% Keep the historical signed-distance convention while sharing intersection math.
dEdge1 = dVertex1 - dVertex0;
dEdge2 = dVertex2 - dVertex0;
dCrossEdge2 = cross(dDirection, dEdge2);
dDet = dot(dEdge1, dCrossEdge2);
dInverseDet = 0;

if abs(dDet) >= 2 * eps
    dInverseDet = 1 / dDet;
end

[bHit, dRange, dU, dV] = IntersectTriangleEdges(dOrigin - dVertex0, dDirection, ...
    dEdge1, dEdge2, dCrossEdge2, dInverseDet, -Inf, Inf);

% Preserve the original point construction for callers requesting the fifth output.
dPoint = zeros(3, 1);
if bHit && nargout >= 5
    dPoint = (1 - dU - dV) * dVertex0 + dU * dVertex1 + dV * dVertex2;
end
end
