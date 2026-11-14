function tests = testBuildSphereApproachScenario
%% SIGNATURE
% tests = testBuildSphereApproachScenario
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Check the analytic sphere-approach scenario: mesh on the sphere with outward triangles, two-body invariants,
% look-at geometry, cosine slew profile, bit-identity of the step on grids that skip the slew, the state at a
% time independent of the other grid times, target rotation at constant inertial rate, and the named input
% errors. MathCore must be on the path (SetupSimGears adds the
% lib/ submodule; MATHCORE_ROOT, when set, is added on top).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    Function-based MATLAB test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-10-2026  Pietro Califano, Claude Opus 5.5  First implementation.
% 29-10-2026  Pietro Califano, Claude Opus 5.5  Test that the state at a time does not depend on the grid.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SetupSimGears, BuildSphereApproachScenario, RotationVectorToDCM, LogMap_SO3toR3 (MathCore).
% -------------------------------------------------------------------------------------------------------------
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
charRepoRoot = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', '..');
testCase.TestData.charOriginalPath = path;
addpath(fullfile(charRepoRoot, 'matlab'));
SetupSimGears;
charMathCoreRoot = getenv('MATHCORE_ROOT');
if ~isempty(charMathCoreRoot)
    addpath(genpath(fullfile(charMathCoreRoot, 'matlab')));
end
cellRequired = {'rv2kepl', 'PropagateKeplerianElems', 'RotationVectorToDCM', 'LogMap_SO3toR3'};
bMissing = cellfun(@(charName) exist(charName, 'file') == 0, cellRequired);
testCase.assertFalse(any(bMissing), sprintf('MathCore functions not on the path: %s', ...
    strjoin(cellRequired(bMissing), ', ')));
% Default scene of the image-motion recordings: nominal initial state, 1 s grid of 121 samples.
testCase.TestData.dPosition0_IN = [0; -300; 3000];
testCase.TestData.dVelocity0_IN = [0.5; 0.2; -2.0];
testCase.TestData.dTime_s = 0:120;
end

function teardownOnce(testCase)
path(testCase.TestData.charOriginalPath);
end

function testMeshIsOutwardConvexHullOnSphere(testCase)
% All vertices lie on the sphere, normals are radial unit vectors and every triangle faces outward. A convex
% hull of M points that all lie on the hull has 2 M - 4 triangles (Euler's formula for a triangulated sphere).
dRadius_m = 500;
strScenario = BuildSphereApproachScenario(testCase.TestData.dTime_s, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, 'dTargetRadius_m', dRadius_m, 'ui32NumMeshPoints', uint32(400));
dVertices_TB = strScenario.strTargetMesh.dVerticesPositions;
i32Triangles = strScenario.strTargetMesh.i32triangVertexPtrs;
testCase.verifySize(dVertices_TB, [3, 400]);
testCase.verifySize(i32Triangles, [3, 2 * 400 - 4]);
testCase.verifyEqual(vecnorm(dVertices_TB, 2, 1), repmat(dRadius_m, 1, 400), 'RelTol', 4 * eps);
testCase.verifyEqual(strScenario.dVertexNormals_TB, dVertices_TB / dRadius_m, 'AbsTol', 4 * eps);
dEdge1 = dVertices_TB(:, i32Triangles(2, :)) - dVertices_TB(:, i32Triangles(1, :));
dEdge2 = dVertices_TB(:, i32Triangles(3, :)) - dVertices_TB(:, i32Triangles(1, :));
dCentroid = (dVertices_TB(:, i32Triangles(1, :)) + dVertices_TB(:, i32Triangles(2, :)) + ...
    dVertices_TB(:, i32Triangles(3, :))) / 3;
testCase.verifyTrue(all(dot(cross(dEdge1, dEdge2), dCentroid, 1) > 0), ...
    'Every convex-hull triangle must be ordered counter-clockwise seen from outside.');
testCase.verifyTrue(isfield(strScenario.strTargetMesh, 'strRayData'));
end

function testTwoBodyConservesEnergyAndAngularMomentum(testCase)
% Bound: the anomaly tolerance 1e-12 rad is a time error of about 1.5e-9 s on this arc, i.e. a relative state
% error near 1e-12; the Cartesian conversion adds rounding of a few eps per operation. 1e-9 relative leaves a
% margin of about 1e3 over both and still detects any propagation defect (a wrong anomaly moves the state by
% metres, i.e. 1e-3 relative).
dGravParam = 69.89;
strScenario = BuildSphereApproachScenario(testCase.TestData.dTime_s, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, 'dGravParam_m3_s2', dGravParam);
dEnergy = 0.5 * vecnorm(strScenario.dVelocity_IN, 2, 1).^2 - dGravParam ./ vecnorm(strScenario.dPosition_IN, 2, 1);
dMomentum = cross(strScenario.dPosition_IN, strScenario.dVelocity_IN, 1);
testCase.verifyEqual(dEnergy, repmat(dEnergy(1), size(dEnergy)), 'RelTol', 1e-9);
% The momentum is compared by norm: its x component is zero for this initial state, so a per-component relative
% tolerance would test rounding noise.
testCase.verifyLessThanOrEqual(max(vecnorm(dMomentum - dMomentum(:, 1), 2, 1)) / norm(dMomentum(:, 1)), 1e-9);
testCase.verifyEqual(strScenario.dPosition_IN(:, 1), testCase.TestData.dPosition0_IN, 'AbsTol', 1e-9);
testCase.verifyEqual(strScenario.dVelocity_IN(:, 1), testCase.TestData.dVelocity0_IN, 'AbsTol', 1e-12);
% Gravity bends the path. To first order in GM the departure from the straight line at time T is
% d(T) = int_0^T (T - s) a(r_line(s)) ds with a(r) = -GM r / |r|^3 (about 6 cm here). The second-order term is
% of relative size GM T^2 / r^3 (4e-5 on this arc) and the trapezoidal error on 20001 nodes is below 1e-8, so
% the propagated departure must match the first-order integral to 1e-3 relative (vector norm).
strLine = BuildSphereApproachScenario(testCase.TestData.dTime_s, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, 'dGravParam_m3_s2', 0);
testCase.verifyEqual(strLine.dPosition_IN, testCase.TestData.dPosition0_IN + ...
    testCase.TestData.dVelocity0_IN * testCase.TestData.dTime_s, 'AbsTol', 0);
dFinalTime = testCase.TestData.dTime_s(end);
dQuadrature_s = linspace(0, dFinalTime, 20001);
dLine_IN = testCase.TestData.dPosition0_IN + testCase.TestData.dVelocity0_IN * dQuadrature_s;
dAcceleration_IN = -dGravParam * dLine_IN ./ vecnorm(dLine_IN, 2, 1).^3;
dFirstOrderDeparture = trapz(dQuadrature_s, (dFinalTime - dQuadrature_s) .* dAcceleration_IN, 2);
dDeparture = strScenario.dPosition_IN(:, end) - strLine.dPosition_IN(:, end);
testCase.verifyLessThanOrEqual(norm(dDeparture - dFirstOrderDeparture) / norm(dFirstOrderDeparture), 1e-3);
end

function testLookAtPointsBoresightAtTarget(testCase)
% Without a slew, z_C points to the target centre, x_C is orthogonal to the up reference and R_IC is a rotation.
dUp_IN = [1; 0; 0];
strScenario = BuildSphereApproachScenario(testCase.TestData.dTime_s, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, 'dUpReference_IN', dUp_IN);
for dSampleIdx = 1:numel(testCase.TestData.dTime_s)
    dDCM = strScenario.dDCM_INfromCAM(:, :, dSampleIdx);
    dPosition = strScenario.dPosition_IN(:, dSampleIdx);
    testCase.verifyEqual(dDCM(:, 3), -dPosition / norm(dPosition), 'AbsTol', 8 * eps);
    testCase.verifyEqual(dot(dDCM(:, 1), dUp_IN), 0, 'AbsTol', 8 * eps);
    testCase.verifyGreaterThan(dot(cross(dUp_IN, dDCM(:, 3)), dDCM(:, 1)), 0);
    testCase.verifyEqual(dDCM' * dDCM, eye(3), 'AbsTol', 16 * eps);
    testCase.verifyEqual(det(dDCM), 1, 'AbsTol', 16 * eps);
end
testCase.verifyEqual(strScenario.dPointingSlewFraction, zeros(1, numel(testCase.TestData.dTime_s)));
end

function testCosineSlewProfileOnDenseGrid(testCase)
% s(tau) is 0 before, 1 after, 1/2 at the centre of the interval; the slew rotation is Exp(s phi_C) and s changes
% between grid times by at most max|s'| dtau = pi / (2 D) dtau.
dRotation_C = deg2rad(1) * [1; 0; 0];
dInterval_s = [60.499; 60.501];
dTime_s = [0:0.5:60, 60.4985:1e-4:60.5015, 61:0.5:62];
strSlew = BuildSphereApproachScenario(dTime_s, testCase.TestData.dPosition0_IN, testCase.TestData.dVelocity0_IN, ...
    'dPointingSlewRotation_C', dRotation_C, 'dPointingSlewInterval_s', dInterval_s);
strLookAt = BuildSphereApproachScenario(dTime_s, testCase.TestData.dPosition0_IN, testCase.TestData.dVelocity0_IN);
dFraction = strSlew.dPointingSlewFraction;
testCase.verifyEqual(dFraction(dTime_s <= dInterval_s(1)), zeros(1, nnz(dTime_s <= dInterval_s(1))));
testCase.verifyEqual(dFraction(dTime_s >= dInterval_s(2)), ones(1, nnz(dTime_s >= dInterval_s(2))));
[~, dCentreIdx] = min(abs(dTime_s - mean(dInterval_s)));
testCase.verifyEqual(dFraction(dCentreIdx), ...
    0.5 * (1 - cos(pi * (dTime_s(dCentreIdx) - dInterval_s(1)) / (dInterval_s(2) - dInterval_s(1)))), 'AbsTol', 1e-12);
testCase.verifyEqual(dFraction(dCentreIdx), 0.5, 'AbsTol', 1e-6);
bDense = dTime_s > 60.4 & dTime_s < 60.6;
dMaxStep = pi / (2 * diff(dInterval_s)) * 1e-4;
testCase.verifyLessThanOrEqual(max(abs(diff(dFraction(bDense)))), dMaxStep * (1 + 1e-6));
for dSampleIdx = 1:numel(dTime_s)
    dRelative = strLookAt.dDCM_INfromCAM(:, :, dSampleIdx)' * strSlew.dDCM_INfromCAM(:, :, dSampleIdx);
    testCase.verifyEqual(LogMap_SO3toR3(dRelative), dFraction(dSampleIdx) * dRotation_C, 'AbsTol', 1e-12);
end
end

function testStepIsBitIdenticalOnGridsSkippingTheSlew(testCase)
% On the 1 s grid with the slew inside (60, 61) s, every sample from 61 s on equals R_IC_lookat Exp(phi_C)
% exactly, the instantaneous step of the image-motion recordings.
dRotation_C = deg2rad(1) * [1; 0; 0];
strSlew = BuildSphereApproachScenario(testCase.TestData.dTime_s, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, 'dPointingSlewRotation_C', dRotation_C, ...
    'dPointingSlewInterval_s', [60.499; 60.501]);
strLookAt = BuildSphereApproachScenario(testCase.TestData.dTime_s, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN);
dExpected = strLookAt.dDCM_INfromCAM;
dExpected(:, :, 62:end) = pagemtimes(dExpected(:, :, 62:end), RotationVectorToDCM(dRotation_C));
testCase.verifyTrue(isequal(strSlew.dDCM_INfromCAM, dExpected));
testCase.verifyTrue(isequal(strSlew.dPosition_IN, strLookAt.dPosition_IN));
end

function testStateAtATimeDoesNotDependOnTheGrid(testCase)
% The 10 ms grid of the gyro edges contains the 1 s truth grid; at the common times the position, velocity and
% camera attitude of the two builds are identical (each time is propagated from the epoch, so no propagation
% error accumulates along a grid). Times from integer nanoseconds, as in GenerateImageMotionRecordingSet.
i64Truth_ns = int64(1000000000) * int64(0:30);
i64Dense_ns = int64(10000000) * int64(0:3000);
dRotation_C = deg2rad(1) * [1; 0; 0];
cellOptions = {'dPointingSlewRotation_C', dRotation_C, 'dPointingSlewInterval_s', [14.499; 14.501]};
strTruth = BuildSphereApproachScenario(double(i64Truth_ns) * 1e-9, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, cellOptions{:});
strDense = BuildSphereApproachScenario(double(i64Dense_ns) * 1e-9, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, cellOptions{:});
dCommonIdx = 1 + 100 * (0:30);
testCase.verifyTrue(isequal(strDense.dPosition_IN(:, dCommonIdx), strTruth.dPosition_IN), sprintf( ...
    'position differs by up to %.3g m', ...
    max(abs(strDense.dPosition_IN(:, dCommonIdx) - strTruth.dPosition_IN), [], 'all')));
testCase.verifyTrue(isequal(strDense.dVelocity_IN(:, dCommonIdx), strTruth.dVelocity_IN));
testCase.verifyTrue(isequal(strDense.dDCM_INfromCAM(:, :, dCommonIdx), strTruth.dDCM_INfromCAM));
end

function testTargetRotatesAtConstantInertialRate(testCase)
% T coincides with I at the epoch and R_IT(t) = Exp(t Omega_I).
dRate_IN = [1e-4; -2e-4; 3e-4];
strScenario = BuildSphereApproachScenario(testCase.TestData.dTime_s, testCase.TestData.dPosition0_IN, ...
    testCase.TestData.dVelocity0_IN, 'dTargetAngVel_IN', dRate_IN);
dDCM_IT = strScenario.strTargetRotation.dDCM_INfromTB;
testCase.verifyEqual(dDCM_IT(:, :, 1), eye(3), 'AbsTol', 4 * eps);
for dSampleIdx = [2, 61, 121]
    testCase.verifyEqual(dDCM_IT(:, :, dSampleIdx), ...
        RotationVectorToDCM(testCase.TestData.dTime_s(dSampleIdx) * dRate_IN), 'AbsTol', 1e-14);
end
end

function testInvalidInputsRaiseNamedErrors(testCase)
dPosition0 = testCase.TestData.dPosition0_IN;
dVelocity0 = testCase.TestData.dVelocity0_IN;
testCase.verifyError(@() BuildSphereApproachScenario(1:10, dPosition0, dVelocity0), ...
    'BuildSphereApproachScenario:InvalidTimeGrid');
testCase.verifyError(@() BuildSphereApproachScenario([0, 2, 1], dPosition0, dVelocity0), ...
    'BuildSphereApproachScenario:InvalidTimeGrid');
testCase.verifyError(@() BuildSphereApproachScenario(0:10, dPosition0, dVelocity0, ...
    'dPointingSlewRotation_C', [0.01; 0; 0], 'dPointingSlewInterval_s', [5; 5]), ...
    'BuildSphereApproachScenario:InvalidSlewInterval');
testCase.verifyError(@() BuildSphereApproachScenario(0:10, dPosition0, dVelocity0, ...
    'dPointingSlewRotation_C', [0.01; 0; 0]), 'BuildSphereApproachScenario:MissingSlewInterval');
testCase.verifyError(@() BuildSphereApproachScenario(0:10, [3000; 0; 0], [0; 0; 0], ...
    'dGravParam_m3_s2', 0), 'BuildSphereApproachScenario:DegenerateLookAt');
end
