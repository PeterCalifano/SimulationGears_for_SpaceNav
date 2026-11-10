function [bHit, dDistance, dIntersectionPoint, ui32TriangleId] = ...
    TraceTriangleRayArrays(ui32TriangleCount, dVertex0, dEdge1, dEdge2, ...
                           dNodeMin, dNodeMax, ui32NodeLeft, ui32NodeRight, ...
                           ui32LeafStart, ui32LeafCount, ui32TriangleOrder, ...
                           bUseBvh, dOrigin, strQuery) %#codegen
%% SIGNATURE
% [bHit, dDistance, dIntersectionPoint, ui32TriangleId] = TraceTriangleRayArrays(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the generic tracer through separate numeric geometry arguments.
% Share this numerical core with TraceTriangleRay. Direct array MEX inputs remove
% struct-field marshalling; generated argument-protection copies can remain.
% Validate geometry before repeated calls. Retain nearest/any-hit queries,
% sidedness, source IDs and the open lower/closed upper query interval.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32TriangleCount       Active source triangle count.
% dVertex0/dEdge1/dEdge2  (3,N) triangle origins and edges in one length unit.
% dNodeMin/dNodeMax       (3,M) conservative node bounds.
% ui32NodeLeft/Right      (1,M) child indices; zero for leaves.
% ui32LeafStart/Count     (1,M) leaf spans in the source permutation.
% ui32TriangleOrder       (1,N) original source IDs in traversal order.
% bUseBvh                 Select BVH traversal; false scans the active triangles.
% dOrigin                 (3,1) ray origin in the stored mesh frame.
% strQuery                Direction, interval, mode, sidedness and optional ray caches.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bHit                    Accepted intersection exists.
% dDistance               Ray parameter; -1 on a miss.
% dIntersectionPoint      (3,1) hit point; zero on a miss.
% ui32TriangleId          Original triangle ID; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Share a numeric core with direct MEX inputs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% IntersectTriangleEdges, IntersectRayBounds (private).
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
    dOrigin            (3, 1) double
    strQuery           (1, 1) struct
end

arguments (Output)
    bHit                (1, 1) logical
    dDistance           (1, 1) double
    dIntersectionPoint  (3, 1) double
    ui32TriangleId      (1, 1) uint32
end

% Keep geometry validation in preparation, then initialize one stable miss result.
bHit = false;
dDistance = -1;
dIntersectionPoint = zeros(3, 1);
ui32TriangleId = uint32(0);
dBestDistance = strQuery.dMaxDistance;
dProjectedOrigin = zeros(2, 1);

if isfield(strQuery, 'dProjectedMin')
    dProjectedOrigin = dOrigin(strQuery.ui8ProjectionAxes) - ...
        strQuery.dProjectionShear * dOrigin(strQuery.ui8DominantAxis);
end
if ui32TriangleCount == 0
    return
end

if ~bUseBvh
    % Simple flat scan of all triangles; ignore BVH nodes and bounds for small meshes.
    ui32IterationCount = ui32TriangleCount;
    if isfield(strQuery, 'ui32CandidateTriangles')
        ui32IterationCount = strQuery.ui32CandidateCount;
    end

    for ui32Iteration = uint32(1):ui32IterationCount

        ui32Triangle = ui32Iteration;
        if isfield(strQuery, 'ui32CandidateTriangles')
            ui32Triangle = strQuery.ui32CandidateTriangles(ui32Iteration);
        end
        if ui32Triangle == strQuery.ui32IgnoreTriangle
            continue
        end

        [bCandidate, dCandidate] = IntersectPreparedTriangle_( ...
            dVertex0, dEdge1, dEdge2, dOrigin, strQuery, ui32Triangle, dBestDistance, dProjectedOrigin);

        if bCandidate && (~bHit || dCandidate < dBestDistance || ...
                (dCandidate == dBestDistance && ui32Triangle < ui32TriangleId))
            bHit = true;
            dBestDistance = dCandidate;
            ui32TriangleId = ui32Triangle;
            if strQuery.bAnyHit
                break
            end
        end
    end

else
    % Use a fixed-size stack to traverse the BVH; order children along the dominant axis.
    dInverseDirection = zeros(3, 1);
    for ui8Axis = uint8(1):uint8(3)
        if strQuery.dDirection(ui8Axis) ~= 0
            dInverseDirection(ui8Axis) = 1 / strQuery.dDirection(ui8Axis);
        end
    end

    [~, dOrderAxis] = max(abs(strQuery.dDirection));
    dOriginSlack = 64 * eps * max(1, max(abs(dOrigin)));

    % uint32 source IDs bound a balanced tree to 32 levels plus two spare entries.
    % Keep this storage fixed for both MATLAB and generated array interfaces.
    ui32Stack = zeros(1, 34, 'uint32');
    ui32Stack(1) = 1;
    ui32StackCount = uint32(1);

    while ui32StackCount > 0

        ui32Node = ui32Stack(ui32StackCount);
        ui32StackCount = ui32StackCount - 1;
        [bBoundsHit, ~] = IntersectRayBounds(dOrigin, strQuery.dDirection, ...
            dNodeMin(:, ui32Node), dNodeMax(:, ui32Node), ...
            strQuery.dMinDistance, dBestDistance, dInverseDirection, dOriginSlack);

        if ~bBoundsHit
            continue
        end

        if ui32LeafCount(ui32Node) > 0

            ui32Start = ui32LeafStart(ui32Node);
            ui32End = ui32Start + ui32LeafCount(ui32Node) - 1;

            for ui32LeafIndex = ui32Start:ui32End

                ui32Triangle = ui32TriangleOrder(ui32LeafIndex);
                if ui32Triangle == strQuery.ui32IgnoreTriangle
                    continue
                end

                [bCandidate, dCandidate] = IntersectPreparedTriangle_( ...
                    dVertex0, dEdge1, dEdge2, dOrigin, strQuery, ui32Triangle, dBestDistance, dProjectedOrigin);

                if bCandidate && (~bHit || dCandidate < dBestDistance || ...
                        (dCandidate == dBestDistance && ui32Triangle < ui32TriangleId))

                    bHit = true;
                    dBestDistance = dCandidate;
                    ui32TriangleId = ui32Triangle;
                    if strQuery.bAnyHit
                        break
                    end
                end
            end
            if bHit && strQuery.bAnyHit
                break
            end
        else
            % Order children along the dominant ray axis; test their bounds when popped.
            ui32Left = ui32NodeLeft(ui32Node);
            ui32Right = ui32NodeRight(ui32Node);

            if strQuery.dDirection(dOrderAxis) > 0
                bLeftFirst = dNodeMin(dOrderAxis, ui32Left) < ...
                    dNodeMin(dOrderAxis, ui32Right);
            else
                bLeftFirst = dNodeMax(dOrderAxis, ui32Left) > ...
                    dNodeMax(dOrderAxis, ui32Right);
            end

            if ~bLeftFirst
                ui32Tmp = ui32Left;
                ui32Left = ui32Right;
                ui32Right = ui32Tmp;
            end

            ui32StackCount = ui32StackCount + 1;
            ui32Stack(ui32StackCount) = ui32Right;
            ui32StackCount = ui32StackCount + 1;
            ui32Stack(ui32StackCount) = ui32Left;
        end
    end
end

% Construct the point only when the caller requests it.
if bHit
    dDistance = dBestDistance;
    if nargout >= 3
        dIntersectionPoint = dOrigin + dDistance * strQuery.dDirection;
    end
end
end

function [bHit, dDistance] = IntersectPreparedTriangle_( ...
    dVertex0, dEdge1, dEdge2, dOrigin, strQuery, ui32Triangle, dMaxDistance, dProjectedOrigin)
%% SIGNATURE
% [bHit, dDistance] = IntersectPreparedTriangle_(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Test one prepared triangle, reusing optional parallel-ray coefficients and
% conservative projected bounds. Keep numerical interval tests in the shared kernel.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dVertex0           (3,N) triangle origins.
% dEdge1/dEdge2      (3,N) prepared triangle edges.
% dOrigin            (3,1) ray origin in the mesh frame.
% strQuery           Query flags, direction, interval and optional direction caches.
% ui32Triangle       Source triangle ID.
% dMaxDistance       Current closed upper distance bound.
% dProjectedOrigin   (2,1) origin in the optional projected coordinate system.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bHit               Triangle intersects the query interval.
% dDistance          Ray parameter; zero on a miss.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-10-2026  Pietro Califano, Codex (GPT-6)  Document the shared prepared-triangle contract.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% IntersectTriangleEdges.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dVertex0    (3, :) double
    dEdge1      (3, :) double
    dEdge2      (3, :) double
    dOrigin (3, 1) double
    strQuery (1, 1) struct
    ui32Triangle (1, 1) uint32
    dMaxDistance (1, 1) double
    dProjectedOrigin (2, 1) double
end

arguments (Output)
    bHit (1, 1) logical
    dDistance (1, 1) double
end

% Reject conservative projected bounds before loading triangle coefficients.
if isfield(strQuery, 'dProjectedMin')
    if any(dProjectedOrigin < strQuery.dProjectedMin(:, ui32Triangle)) || ...
            any(dProjectedOrigin > strQuery.dProjectedMax(:, ui32Triangle))
        bHit = false;
        dDistance = 0;
        return
    end
end

% Reuse direction coefficients for parallel shadow rays; compute only visited
% triangles for ordinary BVH queries.
if isfield(strQuery, 'dCrossEdge2')

    dInverseDet = strQuery.dInverseDet(ui32Triangle);

    if dInverseDet == 0
        bHit = false;
        dDistance = 0;
        return
    end

    dCrossEdge2 = strQuery.dCrossEdge2(:, ui32Triangle);

else

    dTriangleEdge1 = dEdge1(:, ui32Triangle);
    dTriangleEdge2 = dEdge2(:, ui32Triangle);
    dCrossEdge2 = cross(strQuery.dDirection, dTriangleEdge2);
    dDet = dot(dTriangleEdge1, dCrossEdge2);
    dInverseDet = 0;
    if (strQuery.bTwoSided && abs(dDet) >= eps) || ...
            (~strQuery.bTwoSided && dDet >= eps)
        dInverseDet = 1 / dDet;
    end
end

dOriginFromVertex = dOrigin - dVertex0(:, ui32Triangle);

[bHit, dDistance] = IntersectTriangleEdges(dOriginFromVertex, strQuery.dDirection, ...
                    dEdge1(:, ui32Triangle), dEdge2(:, ui32Triangle), ...
                    dCrossEdge2, dInverseDet, strQuery.dMinDistance, dMaxDistance);
end
