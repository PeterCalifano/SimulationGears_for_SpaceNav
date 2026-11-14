function [dDistance, dClosestPoint, ui32TriangleId] = QueryTriangleNearestPoint( ...
    strRayData, dPoint, ui32HintTriangle) %#codegen
%% SIGNATURE
% [dDistance,dClosestPoint,ui32TriangleId] = QueryTriangleNearestPoint( ...
%     strRayData,dPoint,ui32HintTriangle=uint32(0))
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Query unsigned Euclidean distance to the union of the exact prepared triangles. Use the existing 13-field
% BuildTriangleRayData payload without changing geometry, capacities or source IDs. A positive source-triangle
% hint warms the upper bound but cannot change the answer; exact distance ties choose the lowest source ID.
% Traverse the prepared BVH nearest-first using conservative point/AABB lower bounds and the existing fixed
% 34-entry stack limit. Flat scanning supports small fixtures; a production caller can require bUseBvh itself.
% Closest points use face projection plus the three closed segments, including all degenerate segments/points.
% A nonzero scaled cross product admits a thin face without a geometric thickness tolerance. No inside/outside,
% watertightness, sign, temporal clearance or collision claim is made. Validate immutable geometry once with
% ValidateTriangleRayData before repeated queries; this routine checks only schema/header and visited spans.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strRayData       Prepared, validated numeric ray payload; no new fields are required or added.
% dPoint           Finite real 3x1 point in the geometry frame and length units.
% ui32HintTriangle Scalar uint32 source ID in 1:ui32TriangleCount, or uint32(0) for no hint.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDistance        Finite unsigned Euclidean distance in the geometry length units.
% dClosestPoint    3x1 nearest point in the same frame/units.
% ui32TriangleId   Original one-based source triangle ID, with stable lowest-ID exact ties.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Add immutable prepared-mesh nearest-point queries.
% 31-10-2026  Pietro Califano, Claude (Opus 5.5)  Move into SimulationGears from the Eros campaign copy.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildTriangleRayData and ValidateTriangleRayData (preparation only); MATLAB/codegen numerical operations.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strRayData (1,1) struct
    dPoint (3,1) double {mustBeFinite, mustBeReal}
    ui32HintTriangle (1,1) = uint32(0)
end
arguments (Output)
    dDistance (1,1) double
    dClosestPoint (3,1) double
    ui32TriangleId (1,1) uint32
end

% Schema checks inspect only the constant-size header, never the geometry arrays' contents.
if coder.target('MATLAB')
    cellFields = {'ui32TriangleCount', 'dVertex0', 'dEdge1', 'dEdge2', 'dNodeMin', 'dNodeMax', ...
        'ui32NodeLeft', 'ui32NodeRight', 'ui32LeafStart', 'ui32LeafCount', 'ui32TriangleOrder', ...
        'ui32NodeCount', 'bUseBvh'};
    assert(numel(fieldnames(strRayData)) == 13 && all(isfield(strRayData, cellFields)), ...
        'QueryTriangleNearestPoint:InvalidSchema', 'Use the exact prepared 13-field ray payload.');
end

assert(isa(strRayData.ui32TriangleCount, 'uint32') && isscalar(strRayData.ui32TriangleCount), ...
    'QueryTriangleNearestPoint:InvalidGeometry', 'Prepared triangle count must be a scalar uint32.');
ui32TriangleCount = strRayData.ui32TriangleCount;
assert(ui32TriangleCount > 0, 'QueryTriangleNearestPoint:EmptyGeometry', ...
    'Nearest-point distance requires at least one prepared triangle.');
assert(size(strRayData.dVertex0, 1) == 3 && size(strRayData.dVertex0, 2) >= double(ui32TriangleCount) && ...
    isequal(size(strRayData.dVertex0), size(strRayData.dEdge1), size(strRayData.dEdge2)), ...
    'QueryTriangleNearestPoint:InvalidGeometry', 'Prepared triangle arrays have insufficient shared capacity.');

% Avoid the saturating integer conversion that a typed arguments block could apply to a bad caller hint.
assert(isa(ui32HintTriangle, 'uint32') && ui32HintTriangle <= ui32TriangleCount, ...
    'QueryTriangleNearestPoint:InvalidHint', 'Hint must be uint32(0) or a valid source triangle ID.');

% Start from the hint triangle, when given, to tighten the first upper bound
dDistance = Inf;
dClosestPoint = zeros(3, 1);
ui32TriangleId = uint32(0);

if ui32HintTriangle > 0
    [dDistance, dClosestPoint] = ClosestTriangle_(dPoint, ...
        strRayData.dVertex0(:, ui32HintTriangle), strRayData.dEdge1(:, ui32HintTriangle), ...
        strRayData.dEdge2(:, ui32HintTriangle));
    ui32TriangleId = ui32HintTriangle;
end

if ~strRayData.bUseBvh
    % Scan every triangle; on an exact tie keep the lowest source ID
    for ui32Triangle = uint32(1):ui32TriangleCount
        [dCandidate, dCandidatePoint] = ClosestTriangle_(dPoint, strRayData.dVertex0(:, ui32Triangle), ...
            strRayData.dEdge1(:, ui32Triangle), strRayData.dEdge2(:, ui32Triangle));
        if dCandidate < dDistance || (dCandidate == dDistance && ui32Triangle < ui32TriangleId)
            dDistance = dCandidate;
            dClosestPoint = dCandidatePoint;
            ui32TriangleId = ui32Triangle;
        end
    end
else
    ui32NodeCount = strRayData.ui32NodeCount;
    assert(isa(ui32NodeCount, 'uint32') && isscalar(ui32NodeCount) && ui32NodeCount > 0 && ...
        double(ui32NodeCount) <= size(strRayData.dNodeMin, 2), ...
        'QueryTriangleNearestPoint:InvalidBvh', 'BVH traversal requires a prepared nonempty tree.');

    % Traverse the BVH nearest-first with a fixed-size stack of nodes and their distance lower bounds
    ui32Stack = zeros(1, 34, 'uint32');
    dStackLower = zeros(1, 34);
    ui32Stack(1) = uint32(1);
    ui32StackCount = uint32(1);
    dStackLower(1) = BoundsDistance_(dPoint, strRayData.dNodeMin(:, 1), strRayData.dNodeMax(:, 1));

    while ui32StackCount > 0
        ui32Node = ui32Stack(ui32StackCount);
        dLower = dStackLower(ui32StackCount);
        ui32StackCount = ui32StackCount - uint32(1);

        % Keep equality eligible so a hint or traversal order cannot hide a lower-ID exact tie.
        if dLower > dDistance
            continue;
        end

        ui32LeafCount = strRayData.ui32LeafCount(ui32Node);
        if ui32LeafCount > 0
            % Leaf: test its triangles through the source permutation
            ui32Start = strRayData.ui32LeafStart(ui32Node);
            assert(ui32Start >= 1 && double(ui32Start) + double(ui32LeafCount) - 1 <= double(ui32TriangleCount), ...
                'QueryTriangleNearestPoint:InvalidBvh', 'Visited leaf exceeds the active source permutation.');
            ui32End = ui32Start + ui32LeafCount - uint32(1);

            for ui32Leaf = ui32Start:ui32End
                ui32Triangle = strRayData.ui32TriangleOrder(ui32Leaf);
                assert(ui32Triangle >= 1 && ui32Triangle <= ui32TriangleCount, ...
                    'QueryTriangleNearestPoint:InvalidBvh', 'Visited leaf contains an invalid source triangle ID.');
                [dCandidate, dCandidatePoint] = ClosestTriangle_(dPoint, strRayData.dVertex0(:, ui32Triangle), ...
                    strRayData.dEdge1(:, ui32Triangle), strRayData.dEdge2(:, ui32Triangle));
                if dCandidate < dDistance || (dCandidate == dDistance && ui32Triangle < ui32TriangleId)
                    dDistance = dCandidate;
                    dClosestPoint = dCandidatePoint;
                    ui32TriangleId = ui32Triangle;
                end
            end
        else
            % Inner node: order the two children by their distance lower bounds
            ui32Left = strRayData.ui32NodeLeft(ui32Node);
            ui32Right = strRayData.ui32NodeRight(ui32Node);
            assert(ui32Left > ui32Node && ui32Right > ui32Node && ui32Left <= ui32NodeCount && ...
                ui32Right <= ui32NodeCount && ui32Left ~= ui32Right, ...
                'QueryTriangleNearestPoint:InvalidBvh', 'Visited children must follow their unique parent.');
            dLeft = BoundsDistance_(dPoint, strRayData.dNodeMin(:, ui32Left), strRayData.dNodeMax(:, ui32Left));
            dRight = BoundsDistance_(dPoint, strRayData.dNodeMin(:, ui32Right), strRayData.dNodeMax(:, ui32Right));

            ui32Near = ui32Left;
            ui32Far = ui32Right;
            dNear = dLeft;
            dFar = dRight;
            if dRight < dLeft
                ui32Near = ui32Right;
                ui32Far = ui32Left;
                dNear = dRight;
                dFar = dLeft;
            end

            % Push the farther child first; pop the nearer child without a priority queue or geometry copy.
            if dFar <= dDistance
                assert(ui32StackCount < 34, 'QueryTriangleNearestPoint:StackCapacity', ...
                    'Prepared BVH exceeds the validated 34-entry traversal stack.');
                ui32StackCount = ui32StackCount + uint32(1);
                ui32Stack(ui32StackCount) = ui32Far;
                dStackLower(ui32StackCount) = dFar;
            end

            if dNear <= dDistance
                assert(ui32StackCount < 34, 'QueryTriangleNearestPoint:StackCapacity', ...
                    'Prepared BVH exceeds the validated 34-entry traversal stack.');
                ui32StackCount = ui32StackCount + uint32(1);
                ui32Stack(ui32StackCount) = ui32Near;
                dStackLower(ui32StackCount) = dNear;
            end
        end
    end
end

assert(ui32TriangleId > 0 && isfinite(dDistance) && all(isfinite(dClosestPoint)), ...
    'QueryTriangleNearestPoint:NumericalRange', ...
    'No finite nearest point could be represented; do not treat this as clearance.');
end

function dLower = BoundsDistance_(dPoint, dMin, dMax)
%% SIGNATURE
% dLower = BoundsDistance_(dPoint, dMin, dMax)
%% DESCRIPTION
% Use the prepared padded box and subtract a conservative arithmetic guard only from the pruning bound.
% This guard changes traversal work, never the reported triangle distance or exact-tie rule.
%% INPUT
% dPoint, dMin, dMax  Query point and one visited node's prepared bounds.
%% OUTPUT
% dLower              Nonnegative lower bound in geometry length units.
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Bound point/AABB distance conservatively.
%% DEPENDENCIES
% None.
arguments (Input)
    dPoint (3,1) double
    dMin (3,1) double
    dMax (3,1) double
end
arguments (Output)
    dLower (1,1) double
end

assert(all(isfinite(dMin)) && all(isfinite(dMax)) && all(dMin <= dMax), ...
    'QueryTriangleNearestPoint:InvalidBvh', 'Visited node bounds must be finite and ordered.');

dDelta = max(max(dMin - dPoint, dPoint - dMax), 0);
dRoundoff = 64 * eps * max([1; abs(dPoint); abs(dMin); abs(dMax)]);
dLower = max(0, norm(dDelta) - dRoundoff);
end

function [dDistance, dClosest] = ClosestTriangle_(dPoint, dVertex, dEdge1, dEdge2)
%% SIGNATURE
% [dDistance, dClosest] = ClosestTriangle_(dPoint, dVertex, dEdge1, dEdge2)
%% DESCRIPTION
% Minimize over all three closed segments and, when nondegenerate, the projected face interior. Work relative
% to the stored origin and scale orientation predicates instead of subtracting a nearly singular Gram matrix.
% Only an exactly zero computed cross product suppresses the face; no thickness tolerance drops thin faces.
%% INPUT
% dPoint       Query point.
% dVertex      Prepared triangle origin.
% dEdge1/2     Exact stored edges from that origin.
%% OUTPUT
% dDistance    Unsigned distance.
% dClosest     Closest point in the original geometry frame.
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Handle face, segment and point primitives together.
%% DEPENDENCIES
% ClosestSegment_.
arguments (Input)
    dPoint (3,1) double
    dVertex (3,1) double
    dEdge1 (3,1) double
    dEdge2 (3,1) double
end
arguments (Output)
    dDistance (1,1) double
    dClosest (3,1) double
end

dLocal = dPoint - dVertex;
assert(all(isfinite(dLocal)) && all(isfinite(dEdge1)) && all(isfinite(dEdge2)), ...
    'QueryTriangleNearestPoint:NumericalRange', 'Visited geometry or relative query exceeds finite arithmetic.');

% Take the nearest of the three closed edges
[dDistance, dClosestLocal] = ClosestSegment_(dLocal, zeros(3, 1), dEdge1);

[dCandidate, dCandidatePoint] = ClosestSegment_(dLocal, dEdge1, dEdge2);
if dCandidate < dDistance
    dDistance = dCandidate;
    dClosestLocal = dCandidatePoint;
end

[dCandidate, dCandidatePoint] = ClosestSegment_(dLocal, dEdge2, zeros(3, 1));
if dCandidate < dDistance
    dDistance = dCandidate;
    dClosestLocal = dCandidatePoint;
end

% Replace it with the projection on the face plane when that projection lies inside the triangle
dScale = max([abs(dEdge1); abs(dEdge2)]);
if dScale > 0
    dScaledEdge1 = dEdge1 / dScale;
    dScaledEdge2 = dEdge2 / dScale;
    dNormal = cross(dScaledEdge1, dScaledEdge2);
    dNormalLength = norm(dNormal);

    if dNormalLength > 0
        dNormal = dNormal / dNormalLength;
        dProjection = dLocal - (dLocal' * dNormal) * dNormal;

        % This enclosing cube avoids overflow when scaling a far projection beside a tiny triangle.
        if all(abs(dProjection) <= dScale)
            dScaledPoint = dProjection / dScale;
            bInside = dot(cross(dScaledEdge1, dScaledPoint), dNormal) >= 0 && ...
                dot(cross(dScaledEdge2 - dScaledEdge1, dScaledPoint - dScaledEdge1), dNormal) >= 0 && ...
                dot(cross(-dScaledEdge2, dScaledPoint - dScaledEdge2), dNormal) >= 0;

            if bInside
                dCandidate = norm(dLocal - dProjection);
                if dCandidate < dDistance
                    dDistance = dCandidate;
                    dClosestLocal = dProjection;
                end
            end
        end
    end
end

dClosest = dVertex + dClosestLocal;
assert(isfinite(dDistance) && all(isfinite(dClosest)), ...
    'QueryTriangleNearestPoint:NumericalRange', 'Nearest distance/point exceeds finite arithmetic.');
end

function [dDistance, dClosest] = ClosestSegment_(dPoint, dFirst, dSecond)
%% SIGNATURE
% [dDistance, dClosest] = ClosestSegment_(dPoint, dFirst, dSecond)
%% DESCRIPTION
% Project onto a closed segment using its unit direction, retaining point primitives when endpoints coincide.
%% INPUT
% dPoint, dFirst, dSecond  Point and segment endpoints in one local frame.
%% OUTPUT
% dDistance, dClosest      Unsigned distance and closest point.
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Include collinear and collapsed triangle boundaries.
%% DEPENDENCIES
% None.
arguments (Input)
    dPoint (3,1) double
    dFirst (3,1) double
    dSecond (3,1) double
end
arguments (Output)
    dDistance (1,1) double
    dClosest (3,1) double
end

dEdge = dSecond - dFirst;
dLength = norm(dEdge);
dClosest = dFirst;

% Clamp the projection on the segment; a collapsed segment keeps its first endpoint
if dLength > 0
    assert(isfinite(dLength), 'QueryTriangleNearestPoint:NumericalRange', ...
        'Visited segment length exceeds finite arithmetic.');
    dDirection = dEdge / dLength;
    dAlong = (dPoint - dFirst)' * dDirection;
    assert(isfinite(dAlong), 'QueryTriangleNearestPoint:NumericalRange', ...
        'Visited segment projection exceeds finite arithmetic.');

    if dAlong >= dLength
        dClosest = dSecond;
    elseif dAlong > 0
        dClosest = dFirst + dAlong * dDirection;
    end
end

dDistance = norm(dPoint - dClosest);
end
