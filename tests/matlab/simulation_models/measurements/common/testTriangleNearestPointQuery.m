function objTests = testTriangleNearestPointQuery
%% SIGNATURE
% objTests = testTriangleNearestPointQuery
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Tests of the immutable triangle nearest-point API. Use hand-computed face/edge/vertex and
% concave-union cases, thin/degenerate primitives, closed-mesh points and a separate active-set least-squares
% oracle. Build and validate ordinary fixture geometry through the maintained preparation APIs. No asteroid
% assets, rendering, native/MEX builds, temporal monitor or simulation setup is involved.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None. Run matlab/SetupSimGears.m first.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests    Function-based tests.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Specify exact-triangle nearest-point behavior.
% 31-10-2026  Pietro Califano, Claude (Opus 5.5)  Move into SimulationGears from the Eros campaign copy.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% QueryTriangleNearestPoint, BuildTriangleRayData, ValidateTriangleRayData; MATLAB linear algebra only.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testHandComputedFaceEdgesAndVertex(objTestCase)
dTriangle = [0, 2, 0; 0, 0, 2; 0, 0, 0];
dPoints = [.5, 1.5, -1, 1, .5; .5, 1.5, -2, -1, .5; 3, 0, 0, 2, 0];
dExpectedPoints = [.5, 1, 0, 1, .5; .5, 1, 0, 0, .5; 0, 0, 0, 0, 0];
dExpectedDistances = [3, sqrt(.5), sqrt(5), sqrt(5), 0];

for bUseBvh = [false, true]
    strData = Prepare_(reshape(dTriangle, 3, 3, 1), bUseBvh);
    for dPointIdx = 1:size(dPoints, 2)
        for ui32Hint = [uint32(0), uint32(1)]
            [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, dPoints(:, dPointIdx), ui32Hint);
            objTestCase.verifyEqual(dDistance, dExpectedDistances(dPointIdx), 'AbsTol', 1e-12);
            objTestCase.verifyEqual(dClosest, dExpectedPoints(:, dPointIdx), 'AbsTol', 1e-12);
            objTestCase.verifyEqual(ui32Id, uint32(1));
        end
    end
end
end

function testClosedMeshInteriorExteriorAndFarPoint(objTestCase)
dVertices = Cube_();
dPoints = [0, 2, 4, .2; 0, .2, 5, .3; 0, .3, 6, 1];
dExpectedPoints = [0, 1, 1, .2; 0, .2, 1, .3; -1, .3, 1, 1];
dExpectedDistances = [1, 1, sqrt(50), 0];

for bUseBvh = [false, true]
    strData = Prepare_(dVertices, bUseBvh);
    for dPointIdx = 1:size(dPoints, 2)
        [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, dPoints(:, dPointIdx));
        objTestCase.verifyEqual(dDistance, dExpectedDistances(dPointIdx), 'AbsTol', 2e-12);
        objTestCase.verifyEqual(dClosest, dExpectedPoints(:, dPointIdx), 'AbsTol', 2e-12);
        objTestCase.verifyGreaterThan(ui32Id, uint32(0));
    end
end
end

function testExactTieIdAndHintInvarianceAcrossLeaves(objTestCase)
dTriangle = [0, 1, 0; 0, 0, 1; 0, 0, 0];
dHeights = [9:-1:1, 1];
dVertices = repmat(dTriangle, 1, 1, numel(dHeights));
dVertices(3, :, :) = repmat(reshape(dHeights, 1, 1, []), 1, 3, 1);

for bUseBvh = [false, true]
    strData = Prepare_(dVertices, bUseBvh);
    if bUseBvh
        objTestCase.verifyGreaterThan(strData.ui32NodeCount, uint32(1));
    end

    for ui32Hint = [uint32(0), uint32(1), uint32(9), uint32(10)]
        [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, [.2; .2; 0], ui32Hint);
        objTestCase.verifyEqual(dDistance, 1);
        objTestCase.verifyEqual(dClosest, [.2; .2; 1], 'AbsTol', 1e-14);
        objTestCase.verifyEqual(ui32Id, uint32(9));
    end
end
end

function testConcavityIsNotReplacedByConvexHull(objTestCase)
% Three squares form an L; the missing upper-right quadrant remains outside the triangle union.
dVertices = zeros(3, 3, 6);
dOrigins = [0, 1, 0; 0, 0, 1];
for dSquareIdx = 1:3
    dX = dOrigins(1, dSquareIdx);
    dY = dOrigins(2, dSquareIdx);
    dVertices(:, :, 2 * dSquareIdx - 1) = [dX, dX+1, dX+1; dY, dY, dY+1; 0, 0, 0];
    dVertices(:, :, 2 * dSquareIdx) = [dX, dX+1, dX; dY, dY+1, dY+1; 0, 0, 0];
end

for bUseBvh = [false, true]
    strData = Prepare_(dVertices, bUseBvh);
    [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, [1.5; 1.5; .2], uint32(5));
    objTestCase.verifyEqual(dDistance, sqrt(.29), 'AbsTol', 1e-13);
    objTestCase.verifyEqual(dClosest, [1.5; 1; 0], 'AbsTol', 1e-13);
    objTestCase.verifyEqual(ui32Id, uint32(4));
end
end

function testThinNonzeroFacesAreNotDiscarded(objTestCase)
for dHeight = [1e-8, 1e-16, 1e-100, 1e-200]
    dTriangle = [0, 1, 1; 0, 0, dHeight; 0, 0, 0];
    dPoint = [.75; .25*dHeight; 0];
    for bUseBvh = [false, true]
        strData = Prepare_(reshape(dTriangle, 3, 3, 1), bUseBvh);
        [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, dPoint);
        objTestCase.verifyEqual(dDistance, 0);
        objTestCase.verifyEqual(dClosest, dPoint);
        objTestCase.verifyEqual(ui32Id, uint32(1));
    end
end
end

function testDegenerateSegmentsAndPointsRemainEligible(objTestCase)
dCollinear = [0, 1, 3; 0, 0, 0; 0, 0, 0];
dPointTriangle = repmat([2; 0; 0], 1, 3);
dFarTriangle = [0, 2, 0; 0, 0, 2; 5, 5, 5];

for bUseBvh = [false, true]
    % Collinear triangle: the nearest point lies on its longest edge
    strData = Prepare_(cat(3, dCollinear, repmat(dFarTriangle, 1, 1, 9)), bUseBvh);
    [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, [2; 1; 0], uint32(10));
    objTestCase.verifyEqual(dDistance, 1);
    objTestCase.verifyEqual(dClosest, [2; 0; 0]);
    objTestCase.verifyEqual(ui32Id, uint32(1));

    % Triangle collapsed to a point: the nearest point is that point
    strData = Prepare_(cat(3, dPointTriangle, repmat(dFarTriangle, 1, 1, 9)), bUseBvh);
    [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, [0; 1; 0]);
    objTestCase.verifyEqual(dDistance, sqrt(5), 'AbsTol', 1e-14);
    objTestCase.verifyEqual(dClosest, [2; 0; 0]);
    objTestCase.verifyEqual(ui32Id, uint32(1));
end
end

function testMetreKilometreEquivalence(objTestCase)
dVertices = Cube_();
dPoint = [2; .2; .3];

for bUseBvh = [false, true]
    strMetres = Prepare_(dVertices, bUseBvh);
    strKilometres = Prepare_(dVertices / 1000, bUseBvh);
    [dMetres, dPointMetres, ui32Metres] = QueryTriangleNearestPoint(strMetres, dPoint);
    [dKilometres, dPointKm, ui32Km] = QueryTriangleNearestPoint(strKilometres, dPoint / 1000, uint32(12));
    objTestCase.verifyEqual(dKilometres, dMetres / 1000, 'AbsTol', 1e-15);
    objTestCase.verifyEqual(dPointKm, dPointMetres / 1000, 'AbsTol', 1e-15);
    objTestCase.verifyEqual(ui32Km, ui32Metres);
end
end

function testBvhAgainstIndependentActiveSetOracle(objTestCase)
% A deterministic nondegenerate cloud gives an independent exhaustive reference without random draws.
dVertices = zeros(3, 3, 37);
for dTriangleIdx = 1:size(dVertices, 3)
    dOrigin = [3*cos(dTriangleIdx); 2*sin(dTriangleIdx); dTriangleIdx/9];
    dVertices(:, :, dTriangleIdx) = [dOrigin, dOrigin+[.7; .1*sin(dTriangleIdx); .1], ...
        dOrigin+[-.2; .5; .15*cos(dTriangleIdx)]];
end

strData = Prepare_(dVertices, true);
objTestCase.verifyGreaterThan(strData.ui32NodeCount, uint32(1));

for dCaseIdx = 1:17
    dPoint = [1.7*cos(.37*dCaseIdx); 1.2*sin(.73*dCaseIdx); .2*dCaseIdx-1];
    [dExpected, dExpectedPoint, ui32Expected] = Oracle_(dVertices, dPoint);
    [dDistance, dClosest, ui32Id] = QueryTriangleNearestPoint(strData, dPoint, uint32(37));
    objTestCase.verifyEqual(dDistance, dExpected, 'AbsTol', 2e-11);
    objTestCase.verifyEqual(dClosest, dExpectedPoint, 'AbsTol', 2e-10);
    objTestCase.verifyEqual(ui32Id, ui32Expected);
end
end

function testPayloadAndRngAreUnchanged(objTestCase)
strData = Prepare_(Cube_(), true);
strBefore = strData;
strRngBefore = rng;

for dCaseIdx = 1:9
    QueryTriangleNearestPoint(strData, [dCaseIdx/3; sin(dCaseIdx); cos(dCaseIdx)], uint32(12));
end

objTestCase.verifyEqual(strData, strBefore);
objTestCase.verifyEqual(rng, strRngBefore);
objTestCase.verifyNumElements(fieldnames(strData), 13);
end

function testMalformedEmptyAndBadHintsFailExplicitly(objTestCase)
strEmpty = Prepare_(zeros(3, 3, 0), false);
objTestCase.verifyError(@() QueryTriangleNearestPoint(strEmpty, zeros(3, 1)), ...
    'QueryTriangleNearestPoint:EmptyGeometry');

% Hints out of range, negative or not integer
strData = Prepare_(Cube_(), true);
objTestCase.verifyError(@() QueryTriangleNearestPoint(strData, zeros(3, 1), uint32(13)), ...
    'QueryTriangleNearestPoint:InvalidHint');
objTestCase.verifyError(@() QueryTriangleNearestPoint(strData, zeros(3, 1), -1), ...
    'QueryTriangleNearestPoint:InvalidHint');
objTestCase.verifyError(@() QueryTriangleNearestPoint(strData, zeros(3, 1), 1.5), ...
    'QueryTriangleNearestPoint:InvalidHint');

% Malformed payloads
objTestCase.verifyError(@() QueryTriangleNearestPoint(rmfield(strData, 'dEdge2'), zeros(3, 1)), ...
    'QueryTriangleNearestPoint:InvalidSchema');

strBad = strData;
strBad.ui32TriangleCount = uint32(13);
objTestCase.verifyError(@() QueryTriangleNearestPoint(strBad, zeros(3, 1)), ...
    'QueryTriangleNearestPoint:InvalidGeometry');

strBad = strData;
strBad.ui32NodeCount = uint32(0);
objTestCase.verifyError(@() QueryTriangleNearestPoint(strBad, zeros(3, 1)), ...
    'QueryTriangleNearestPoint:InvalidBvh');

strBad = strData;
strBad.ui32NodeLeft(1) = uint32(1);
objTestCase.verifyError(@() ValidateTriangleRayData(strBad), 'ValidateTriangleRayData:Topology');
objTestCase.verifyError(@() QueryTriangleNearestPoint(strBad, zeros(3, 1)), ...
    'QueryTriangleNearestPoint:InvalidBvh');

objTestCase.verifyError(@() QueryTriangleNearestPoint(strData, [NaN; 0; 0]), 'MATLAB:validators:mustBeFinite');
end

function strData = Prepare_(dVertices, bUseBvh)
%% SIGNATURE
% strData = Prepare_(dVertices, bUseBvh)
%% DESCRIPTION
% Use the maintained preparation and validation boundary; validation is not moved into every query.
%% INPUT
% dVertices  Explicit fixture triangle vertices.
% bUseBvh    Fixture traversal selection.
%% OUTPUT
% strData    Existing 13-field numeric payload.
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Reuse the owning geometry APIs.
%% DEPENDENCIES
% BuildTriangleRayData, ValidateTriangleRayData.
arguments (Input)
    dVertices (3,3,:) double
    bUseBvh (1,1) logical
end
arguments (Output)
    strData (1,1) struct
end

strData = BuildTriangleRayData(dVertices, bUseBvh);
ValidateTriangleRayData(strData);
end

function dTriangles = Cube_()
%% SIGNATURE
% dTriangles = Cube_()
%% DESCRIPTION
% Triangulate the six exact faces of [-1,1]^3; no fitted sphere or ellipsoid enters the oracle.
%% INPUT
% None.
%% OUTPUT
% dTriangles  3x3x12 explicit closed-cube face vertices.
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Supply hand-checkable closed geometry.
%% DEPENDENCIES
% None.
arguments (Input)
end
arguments (Output)
    dTriangles (3,3,:) double
end

dVertices = [-1, 1, 1, -1, -1, 1, 1, -1; -1, -1, 1, 1, -1, -1, 1, 1; -1, -1, -1, -1, 1, 1, 1, 1];
ui32Faces = uint32([1, 2, 3; 1, 3, 4; 5, 7, 6; 5, 8, 7; 1, 5, 6; 1, 6, 2; ...
    2, 6, 7; 2, 7, 3; 3, 7, 8; 3, 8, 4; 4, 8, 5; 4, 5, 1]);

dTriangles = zeros(3, 3, 12);
for ui32Face = uint32(1):uint32(12)
    dTriangles(:, :, ui32Face) = dVertices(:, ui32Faces(ui32Face, :));
end
end

function [dBest, dClosest, ui32Id] = Oracle_(dTriangles, dPoint)
%% SIGNATURE
% [dBest, dClosest, ui32Id] = Oracle_(dTriangles, dPoint)
%% DESCRIPTION
% Exhaustively enumerate the seven nonempty vertex active sets per triangle. Solve a small equality-constrained
% affine least-squares system, admitting only nonnegative barycentric weights. This is independent of the
% production face-projection/segment algorithm. Use well-conditioned cloud fixtures here; thin and degenerate
% primitives have their separate exact analytical tests instead of relying on rank-truncated linear algebra.
%% INPUT
% dTriangles  Fixture vertices, independent of the prepared BVH and source permutation.
% dPoint      Query point.
%% OUTPUT
% dBest, dClosest, ui32Id  Exhaustive nearest distance, point and original triangle ID.
%% CHANGELOG
% 30-10-2026  Pietro Califano, Codex (GPT-6.1)  Add an independent constrained least-squares reference.
%% DEPENDENCIES
% MATLAB rank and backslash; no optimization toolbox.
arguments (Input)
    dTriangles (3,3,:) double
    dPoint (3,1) double
end
arguments (Output)
    dBest (1,1) double
    dClosest (3,1) double
    ui32Id (1,1) uint32
end

dBest = Inf;
dClosest = nan(3, 1);
ui32Id = uint32(0);

for ui32Triangle = uint32(1):uint32(size(dTriangles, 3))
    for ui8Subset = uint8(1):uint8(7)
        % Closest point of the affine hull of the active vertices (KKT system of the least-squares problem)
        bActive = logical(bitget(ui8Subset, 1:3));
        dVertices = dTriangles(:, bActive, ui32Triangle);
        dLocal = dVertices - dPoint;
        dCount = size(dLocal, 2);
        dSystem = [dLocal'*dLocal, ones(dCount, 1); ones(1, dCount), 0];
        if rank(dSystem) < dCount + 1
            continue;
        end

        % Keep the solution only when it lies inside the triangle (nonnegative barycentric weights)
        dSolution = dSystem \ [zeros(dCount, 1); 1];
        dWeights = dSolution(1:dCount);
        if any(dWeights < 0)
            continue;
        end

        dCandidatePoint = dVertices * dWeights;
        dCandidate = norm(dPoint - dCandidatePoint);
        if dCandidate < dBest || (dCandidate == dBest && ui32Triangle < ui32Id)
            dBest = dCandidate;
            dClosest = dCandidatePoint;
            ui32Id = ui32Triangle;
        end
    end
end
end
