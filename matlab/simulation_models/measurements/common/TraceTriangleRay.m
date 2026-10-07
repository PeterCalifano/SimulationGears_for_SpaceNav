function [bHit, dDistance, dIntersectionPoint, ui32TriangleId] = TraceTriangleRay(strRayData, dOrigin, strQuery) %#codegen
%% SIGNATURE
% [bHit, dDistance, dIntersectionPoint, ui32TriangleId] = TraceTriangleRay(strRayData, dOrigin, strQuery)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Trace a prepared mesh using an exact flat scan or bounded binary BVH.
% Adapt RCS-1 nearest-hit traversal (9b6a2d47) to mutable numeric geometry,
% bounded-depth storage and application-independent query intervals.
% Use any-hit for occlusion and nearest-hit for ranges. Preserve source triangle
% IDs and choose the lowest source ID for exactly tied nearest distances.
% Return ray parameters; these equal lengths only for unit directions.
% Supply optional cached direction coefficients for repeated parallel rays.
% Example: strQuery = struct('dDirection',[0;0;-1],'bAnyHit',false, ...
%     'bTwoSided',true,'dMinDistance',0,'dMaxDistance',Inf,'ui32IgnoreTriangle',uint32(0));
% [bHit,dRange] = TraceTriangleRay(strData,[0.2;0.2;1],strQuery);
% Output: true and 1 for a triangle in the XY plane containing [0.2;0.2;0].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strRayData       Numeric payload from BuildTriangleRayData.
% dOrigin          (3,1) ray origin in the mesh frame [length].
% strQuery         Direction, any-hit/sidedness flags, interval and ignored ID.
%                  Optional dCrossEdge2 and dInverseDet arrays cache direction data.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bHit             Intersection exists within the supplied interval.
% dDistance        Ray parameter; -1 on a miss.
% dIntersectionPoint (3,1) hit position [length]; zero on a miss.
% ui32TriangleId   Original one-based triangle ID; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% 07-10-2026  Pietro Califano, Codex (GPT-6)  Reuse RCS-1 direction terms and conservative ray bounds.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% TraceTriangleRayArrays.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strRayData (1, 1) struct
    dOrigin (3, 1) double
    strQuery (1, 1) struct
end

arguments (Output)
    bHit (1, 1) logical
    dDistance (1, 1) double
    dIntersectionPoint (3, 1) double
    ui32TriangleId (1, 1) uint32
end

[bHit, dDistance, dIntersectionPoint, ui32TriangleId] = ...
    TraceTriangleRayArrays(strRayData.ui32TriangleCount, ...
                          strRayData.dVertex0, strRayData.dEdge1, strRayData.dEdge2, ...
                          strRayData.dNodeMin, strRayData.dNodeMax, ...
                          strRayData.ui32NodeLeft, strRayData.ui32NodeRight, ...
                          strRayData.ui32LeafStart, strRayData.ui32LeafCount, ...
                          strRayData.ui32TriangleOrder, strRayData.bUseBvh, dOrigin, strQuery);
end
