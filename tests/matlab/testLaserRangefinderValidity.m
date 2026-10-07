function objTests = testLaserRangefinderValidity
%% SIGNATURE
% objTests = testLaserRangefinderValidity
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify LiDAR validity bounds, misses and nearest forward returns.
% Exercise raw and prepared meshes independently of triangle order and the
% nominal target centre. Preserve signed semantics in the two-sided primitive.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-08-2026  Pietro Califano, Codex gpt-5.6     First implementation.
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Check nearest returns, biased endpoints and misses.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% LaserRangefinderModel, BuildTriangleRayData,
% RayTwoSidedTriangleIntersection_MollerTrembore.
% -------------------------------------------------------------------------------------------------------------

% MATLAB functiontests rejects arguments blocks in the suite and test callbacks.
objTests = functiontests(localfunctions);
end

function setupOnce(testCase)
charRepoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(charRepoRoot, 'matlab', 'SetupSimGears.m'));
testCase.TestData.charRepoRoot = charRepoRoot;
end

function testRejectsRangeAboveUpperBound(testCase)
strTargetModelData = BuildSingleTriangleTarget_();

[dMeasuredRange, bIntersection, bValid] = LaserRangefinderModel( ...
    strTargetModelData, [1.0; 0.0; 0.0], [-10.0; 0.0; 0.0], 0.0, 0.0, ...
    zeros(3,1), [0.0; 5.0], false, false, true);

verifyTrue(testCase, bIntersection);
verifyGreaterThan(testCase, dMeasuredRange, 5.0);
verifyFalse(testCase, bValid);
end

function testRejectsRangeBelowLowerBound(testCase)
strTargetModelData = BuildSingleTriangleTarget_();

[dMeasuredRange, bIntersection, bValid] = LaserRangefinderModel( ...
    strTargetModelData, [1.0; 0.0; 0.0], [-10.0; 0.0; 0.0], 0.0, 0.0, ...
    zeros(3,1), [9.5; 20.0], false, false, true);

verifyTrue(testCase, bIntersection);
verifyLessThan(testCase, dMeasuredRange, 9.5);
verifyFalse(testCase, bValid);
end

function strTargetModelData = BuildSingleTriangleTarget_()
%% SIGNATURE
% strTargetModelData = BuildSingleTriangleTarget_()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Create one triangle intersecting the +X sensor ray at x=-1.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strTargetModelData  Raw mesh using the public signed-index schema.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Document the analytical range fixture.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Output)
    strTargetModelData (1, 1) struct
end

strTargetModelData = struct();
strTargetModelData.i32triangVertexPtrs = int32([1; 2; 3]);
strTargetModelData.dVerticesPositions = [-1.0, -1.0, -1.0; ...
                                         -1.0,  1.0,  0.0; ...
                                         -1.0, -1.0,  1.0];
end

function testNearestReturnForBothMeshSchemas_(objTestCase)
%% SIGNATURE
% testNearestReturnForBothMeshSchemas_(objTestCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Require the nearest forward surface when a farther record comes first and
% the nominal mesh centre lies behind the sensor. Check raw and prepared inputs.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objTestCase  MATLAB assertion owner.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; verify the complete deterministic sensor return.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Cover nearest-hit source interfaces.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSingleTriangleTarget_, BuildTriangleRayData, LaserRangefinderModel.
% -------------------------------------------------------------------------------------------------------------

% Keep the nearer face after a farther face and include a backward surface.
strMesh = BuildSingleTriangleTarget_();
dTriangle = strMesh.dVerticesPositions;
dFaces = cat(3, dTriangle + [4; 0; 0], dTriangle, dTriangle - [11; 0; 0]);
strMesh.dVerticesPositions = reshape(dFaces, 3, []);
strMesh.i32triangVertexPtrs = reshape(int32(1:9), 3, []);

for bPrepared = [false, true]
    strSensorMesh = strMesh;
    if bPrepared
        strSensorMesh = struct('strRayData', BuildTriangleRayData(dFaces, true));
    end
    [dRange, bHit, bValid, dPoint, dError] = LaserRangefinderModel( ...
        strSensorMesh, [1; 0; 0], [-10; 0; 0], 0, 0, [-20; 0; 0], ...
        [0; 20], false, true, true);

    objTestCase.verifyTrue(bHit && bValid);
    objTestCase.verifyEqual(dRange, 9);
    objTestCase.verifyEqual(dPoint, [-1; 0; 0]);
    objTestCase.verifyEqual(dError, 0);
end
end

function testIncludesBiasedIntervalEndpoints_(objTestCase)
%% SIGNATURE
% testIncludesBiasedIntervalEndpoints_(objTestCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Include both range endpoints after applying the configured sensor bias.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objTestCase  MATLAB assertion owner.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; verify biased ranges at both inclusive endpoints.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Check validity after realized sensor error.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSingleTriangleTarget_, LaserRangefinderModel.
% -------------------------------------------------------------------------------------------------------------

strMesh = BuildSingleTriangleTarget_();
for dBias = [1, 2]
    [dRange, bHit, bValid, ~, dError] = LaserRangefinderModel( ...
        strMesh, [1; 0; 0], [-10; 0; 0], 0, dBias, zeros(3, 1), ...
        [10; 11], true, false, true);

    objTestCase.verifyTrue(bHit && bValid);
    objTestCase.verifyEqual(dRange, 9 + dBias);
    objTestCase.verifyEqual(dError, dBias);
end
end

function testMissRemainsInvalidWithoutChecks_(objTestCase)
%% SIGNATURE
% testMissRemainsInvalidWithoutChecks_(objTestCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Reject a geometric miss when range checks are disabled and leave the random
% stream unchanged even when sensor noise and bias are enabled.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objTestCase  MATLAB assertion owner.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; verify exact miss outputs and the unchanged random state.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Reject misses before consuming sensor noise.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSingleTriangleTarget_, LaserRangefinderModel.
% -------------------------------------------------------------------------------------------------------------

strMesh = BuildSingleTriangleTarget_();
strInitialRng = rng;
[dRange, bHit, bValid, dPoint, dError] = LaserRangefinderModel( ...
    strMesh, [-1; 0; 0], [-10; 0; 0], 100, 100, zeros(3, 1), ...
    [0; 20], true, false, false);

objTestCase.verifyFalse(bHit || bValid);
objTestCase.verifyEqual(dRange, -1);
objTestCase.verifyEqual(dPoint, zeros(3, 1));
objTestCase.verifyEqual(dError, 0);
objTestCase.verifyEqual(rng, strInitialRng);
end

function testTwoSidedPrimitiveKeepsSignedRange_(objTestCase)
%% SIGNATURE
% testTwoSidedPrimitiveKeepsSignedRange_(objTestCase)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Preserve the legacy infinite-line hit and its negative parameter while the
% public rangefinder separately requires a forward intersection.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objTestCase  MATLAB assertion owner.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; verify the signed range and reconstructed point.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Retain public two-sided primitive semantics.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildSingleTriangleTarget_, RayTwoSidedTriangleIntersection_MollerTrembore.
% -------------------------------------------------------------------------------------------------------------

strMesh = BuildSingleTriangleTarget_();
dVertices = strMesh.dVerticesPositions;
[bHit, ~, ~, dRange, dPoint] = RayTwoSidedTriangleIntersection_MollerTrembore( ...
    [-10; 0; 0], [-1; 0; 0], dVertices(:, 1), dVertices(:, 2), dVertices(:, 3));

objTestCase.verifyTrue(bHit);
objTestCase.verifyEqual(dRange, -9);
objTestCase.verifyEqual(dPoint, [-1; 0; 0]);
end
