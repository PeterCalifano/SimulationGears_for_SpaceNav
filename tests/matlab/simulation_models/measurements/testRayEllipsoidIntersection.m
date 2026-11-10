function tests = testRayEllipsoidIntersection
%% SIGNATURE
% tests = testRayEllipsoidIntersection
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Check ray/ellipsoid intersection behavior and origin/attitude Jacobians.
% Central differences perturb the input-frame origin or apply a positive local
% TF rotation at the estimated attitude. Cover entry/exit roots, nonzero attitude
% corrections, displaced centres, spheres, independent derivative flags and misses.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    Function-based MATLAB test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 09-09-2026  Pietro Califano, Codex gpt-6    Expand coverage with MathCore differentiation utilities.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SetupSimGears, RayEllipsoidIntersection, ComputeFiniteDiffJacobian, RotationVectorToDCM.
% -------------------------------------------------------------------------------------------------------------
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
charRepoRoot = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', '..');
testCase.TestData.charOriginalPath = path;
addpath(fullfile(charRepoRoot, 'matlab'));
SetupSimGears;
end

function teardownOnce(testCase)
path(testCase.TestData.charOriginalPath);
end

function testSphereFromOutside(testCase)

dOrigin = [0; 0; 3];
dDirection = [0; 0; -1];

dCentre = zeros(3,1);
dInvDiag = ones(3,1); % Radius = 1

[bHit, dIntersectDist, bFailure, dIntersectPoint] = ...
    RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag);

testCase.verifyTrue(bHit);
testCase.verifyFalse(bFailure);
testCase.verifyEqual(dIntersectDist, 2, 'AbsTol', 1e-12);
testCase.verifyEqual(dIntersectPoint, [0; 0; 1], 'AbsTol', 1e-12);

end

function testSphereFromInside(testCase)

dOrigin = [0; 0; 0];
dDirection = [1; 0; 0];
dCentre = zeros(3,1);
dInvDiag = ones(3,1);

[bHit, dIntersectDist, bFailure] = RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag);

testCase.verifyTrue(bHit);
testCase.verifyFalse(bFailure);
testCase.verifyEqual(dIntersectDist, 1, 'AbsTol', 1e-12);
end

function testMissedIntersection(testCase)

dOrigin = [0; 0; 3];
dDirection = [0; 0; 1]; % Pointing away from sphere
dCentre = zeros(3,1);
dInvDiag = ones(3,1);

[bHit, dIntersectDist, bFailure] = RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag);

testCase.verifyFalse(bHit);
testCase.verifyFalse(bFailure);
testCase.verifyEqual(dIntersectDist, 0, 'AbsTol', 1e-12);
end

function testTangentialContactSetsFailure(testCase)

dOrigin = [1; 0; 0];
dDirection = [0; 1; 0]; % Tangent on unit sphere
dCentre = zeros(3,1);
dInvDiag = ones(3,1);

[bHit, dIntersectDist, bFailure] = RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag);

testCase.verifyFalse(bHit); % Consider tangency as invalid
testCase.verifyTrue(bFailure); % Tangency -> ill-conditioned jacobians
testCase.verifyEqual(dIntersectDist, 0, 'AbsTol', 1e-12);
end

function testRotatedEllipsoidMatchesAnalytic(testCase)

dOrigin = [2; -0.5; 1.2];
dDirection = NormalizeVec_([-1; 0.1; -0.2]);
dCentre = [0.3; -0.1; 0.2];
dInvDiag = [1/9; 1/4; 1/2.25];

dR = RotFromAxis_([0; 0; 1], deg2rad(20)) * RotFromAxis_([0; 1; 0], deg2rad(15));

[bHit, dIntersectDist, bFailure, dIntersecPoint] = ...
    RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag, dR, dR);
testCase.verifyTrue(bHit);
testCase.verifyFalse(bFailure);

dExpectedDist = SolveRayEllipsoidDistance_(dR * dOrigin, dR * dDirection, dR * dCentre, dInvDiag);

testCase.verifyEqual(dIntersectDist, dExpectedDist, 'AbsTol', 1e-10);
testCase.verifyEqual(dIntersecPoint, dR * (dOrigin + dDirection * dIntersectDist), 'AbsTol', 1e-10);
end

function testJacobiansAgainstFiniteDiff_Sphere(testCase)

% Build test case
dOrigin_TB = [4; -1; 0.5];
dDirection_TB = NormalizeVec_([-1; 0.2; -0.1]);
dCentre_TB = [0.5; -0.25; 0.2];
dRtrue_TBfromW = RotFromAxis_([0; 0; 1], deg2rad(25));

dRadius = 2;
dInvDiag_TB = 1/dRadius^2 * ones(1,3);

% Intersect
[bHit, dIntersectDist, bFailure, ~, dJacOrigin, dJacAtt] = ...
    RayEllipsoidIntersection(dRtrue_TBfromW' * dOrigin_TB, ...
        dRtrue_TBfromW' * dDirection_TB, dRtrue_TBfromW' * dCentre_TB, dInvDiag_TB, eye(3), eye(3));

% Intersect check (should not change)
[~, dIntersectDist_check, ~, ~, dJacOrigin_check] = ...
    RayEllipsoidIntersection(dRtrue_TBfromW' * dOrigin_TB, ...
        dRtrue_TBfromW' * dDirection_TB, dRtrue_TBfromW' * dCentre_TB, ...
        dInvDiag_TB, dRtrue_TBfromW, dRtrue_TBfromW);

% Assert the two cases are equal (i.e. rotation does not matter)
testCase.verifyEqual(dIntersectDist, dIntersectDist_check, 'AbsTol', 1e-6);
testCase.verifyEqual(dJacOrigin, dJacOrigin_check, 'AbsTol', 1e-6);
testCase.verifyEqual(dJacAtt, zeros(1,3), 'AbsTol', 1e-14);

% Checks
testCase.verifyTrue(bHit);
testCase.verifyFalse(bFailure);
testCase.verifyGreaterThan(dIntersectDist, 0);

dFdmJacOrigin = ComputeFiniteDiffJacobian(@(origin) EvalDistanceWithOrigin_(origin, ...
    dRtrue_TBfromW' * dDirection_TB, dRtrue_TBfromW' * dCentre_TB, ...
    dInvDiag_TB, dRtrue_TBfromW), dRtrue_TBfromW' * dOrigin_TB, 1e-6);
% Check position jacobian
testCase.verifyEqual(dJacOrigin, dFdmJacOrigin, 'AbsTol', 5e-6);
end

function testJacobiansAgainstFiniteDiff_EllipseIdentityRot(testCase)

% Build test case
dOrigin_TB = [4; -1; 0.5];
dDirection_TB = NormalizeVec_([-1; 0.2; -0.1]);
dCentre_TB = [0.5; -0.25; 0.2];
dInvDiag_TB = [1/9; 1/4; 1/1.44];

% Intersect
[bHit, dIntersectDist, bFailure, ~, dJacOrigin, dJacAtt] = ...
    RayEllipsoidIntersection(dOrigin_TB, dDirection_TB, dCentre_TB, dInvDiag_TB);

% Checks
testCase.verifyTrue(bHit);
testCase.verifyFalse(bFailure);
testCase.verifyGreaterThan(dIntersectDist, 0);

dFdmJacOrigin = ComputeFiniteDiffJacobian(@(origin) EvalDistanceWithOrigin_(origin, ...
    dDirection_TB, dCentre_TB, dInvDiag_TB, eye(3)), dOrigin_TB, 1e-6);

% Check position jacobian
testCase.verifyEqual(dJacOrigin, dFdmJacOrigin, 'AbsTol', 5e-6);

% Perturb the target-side rotation while holding the input-frame ray fixed.
dFdmJacAtt = ComputeFiniteDiffJacobian(@(theta) EvalDistanceWithAttErr_(theta, ...
    dOrigin_TB, dDirection_TB, dCentre_TB, dInvDiag_TB, eye(3)), zeros(3,1), 1e-6);
testCase.verifyEqual(dJacAtt, dFdmJacAtt, 'AbsTol', 5e-6);

end

function testJacobiansAgainstFiniteDiff_GeneralCase(testCase)

% Build test case
dOrigin_TB = [4; -1; 0.5];
dDirection_TB = NormalizeVec_([-1; 0.2; -0.1]);
dCentre_TB = [0.5; -0.25; 0.2];
dRtrue_TBfromW = RotFromAxis_([0; 0; 1], deg2rad(25));

dInvDiag_TB = [1/9; 1/4; 1/1.44];

% Intersect
[bHit, dIntersectDist, bFailure, ~, dJacOrigin, dJacAtt] = ...
    RayEllipsoidIntersection(dRtrue_TBfromW' * dOrigin_TB, ...
        dRtrue_TBfromW' * dDirection_TB, dRtrue_TBfromW' * dCentre_TB, ...
        dInvDiag_TB, dRtrue_TBfromW, dRtrue_TBfromW);

% Checks
testCase.verifyTrue(bHit);
testCase.verifyFalse(bFailure);
testCase.verifyGreaterThan(dIntersectDist, 0);

dFdmJacOrigin = ComputeFiniteDiffJacobian(@(origin) EvalDistanceWithOrigin_(origin, ...
    dRtrue_TBfromW' * dDirection_TB, dRtrue_TBfromW' * dCentre_TB, ...
    dInvDiag_TB, dRtrue_TBfromW), dRtrue_TBfromW' * dOrigin_TB, 1e-6);

% Test jacobian of position
testCase.verifyEqual(dJacOrigin, dFdmJacOrigin, 'AbsTol', 5e-6);

% Perturb the same input-frame geometry passed to the analytic evaluation.
dFdmJacAtt = ComputeFiniteDiffJacobian(@(theta) EvalDistanceWithAttErr_(theta, ...
    dRtrue_TBfromW' * dOrigin_TB, dRtrue_TBfromW' * dDirection_TB, ...
    dRtrue_TBfromW' * dCentre_TB, dInvDiag_TB, dRtrue_TBfromW), zeros(3,1), 1e-6);
testCase.verifyEqual(dJacAtt, dFdmJacAtt, 'AbsTol', 5e-6);

end

function testNonzeroCorrectionAndIndependentJacobianFlags(testCase)
dOrigin = [4; -1; 0.5];
dDirection = NormalizeVec_([-1; 0.2; -0.1]);
dCentre = [0.5; -0.25; 0.2];
dInvDiag = [1/9; 1/4; 1/1.44];
dReference = RotFromAxis_([1; -2; 3], 0.4);
dEstimate = RotFromAxis_([2; 1; -1], 0.15) * dReference;

% Keep the ray geometry fixed while the estimated attitude differs from its reference.
dOrigin = dReference' * dOrigin;
dDirection = dReference' * dDirection;
dCentre = dReference' * dCentre;
[bHit, dDistance, bFailure, ~, dJacOrigin, dJacAtt] = RayEllipsoidIntersection( ...
    dOrigin, dDirection, dCentre, dInvDiag, dReference, dEstimate);
testCase.assertTrue(bHit && ~bFailure);

for dStep = [1e-5, 1e-6]
    dNumericOrigin = ComputeFiniteDiffJacobian(@(dPosition) EvalDistanceWithOrigin_( ...
        dPosition, dDirection, dCentre, dInvDiag, dEstimate), dOrigin, dStep);
    dNumericAtt = ComputeFiniteDiffJacobian(@(dAngle) EvalDistanceWithAttErr_( ...
        dAngle, dOrigin, dDirection, dCentre, dInvDiag, dEstimate), zeros(3,1), dStep);
    testCase.verifyEqual(dJacOrigin, dNumericOrigin, 'AbsTol', 5e-8);
    testCase.verifyEqual(dJacAtt, dNumericAtt, 'AbsTol', 5e-8);
end

% Either derivative can be requested on its own without changing the intersection.
bFlagCases = [false, false; true, false; false, true];
for ui32Case = 1:size(bFlagCases,1)
    bFlags = bFlagCases(ui32Case,:);
    [bHitFlag, dDistanceFlag, bFailureFlag, ~, dOriginFlag, dAttFlag] = ...
        RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag, ...
            dReference, dEstimate, bFlags);
    testCase.verifyTrue(bHitFlag && ~bFailureFlag);
    testCase.verifyEqual(dDistanceFlag, dDistance);
    testCase.verifyEqual(dOriginFlag, double(bFlags(1))*dJacOrigin, 'AbsTol', 1e-14);
    testCase.verifyEqual(dAttFlag, double(bFlags(2))*dJacAtt, 'AbsTol', 1e-14);
end
end

function testSphereAttitudeDerivativeIsZero(testCase)
dOrigin = [4; -1; 0.5];
dDirection = NormalizeVec_([-1; 0.2; -0.1]);
dCentre = [0.5; -0.25; 0.2];
dEstimate = RotFromAxis_([1; -2; 3], 0.7);
[bHit, ~, bFailure, ~, ~, dJacAtt] = RayEllipsoidIntersection( ...
    dOrigin, dDirection, dCentre, ones(3,1)/4, eye(3), dEstimate);
testCase.assertTrue(bHit && ~bFailure);
testCase.verifyEqual(dJacAtt, zeros(1,3), 'AbsTol', 1e-14);
end

function testInteriorExitJacobians(testCase)
dOrigin = [0.3; -0.1; 0.2];
dDirection = NormalizeVec_([1; 0.5; 0.7]);
dCentre = [0.1; 0.2; -0.3];
dInvDiag = [1/9; 1/4; 1/1.44];
dEstimate = RotFromAxis_([1; -2; 3], 0.5);
[bHit, ~, bFailure, ~, dJacOrigin, dJacAtt] = RayEllipsoidIntersection( ...
    dOrigin, dDirection, dCentre, dInvDiag, eye(3), dEstimate);
testCase.assertTrue(bHit && ~bFailure);
dNumericOrigin = ComputeFiniteDiffJacobian(@(dPosition) EvalDistanceWithOrigin_( ...
    dPosition, dDirection, dCentre, dInvDiag, dEstimate), dOrigin, 1e-6);
dNumericAtt = ComputeFiniteDiffJacobian(@(dAngle) EvalDistanceWithAttErr_( ...
    dAngle, dOrigin, dDirection, dCentre, dInvDiag, dEstimate), zeros(3,1), 1e-6);
testCase.verifyEqual(dJacOrigin, dNumericOrigin, 'AbsTol', 5e-8);
testCase.verifyEqual(dJacAtt, dNumericAtt, 'AbsTol', 5e-8);
end

function testMissAndInvalidRayReturnZeroJacobians(testCase)
dOrigins = [3,0,0; 0,0,0; 0,3,3];
dDirections = [0,0,0; 1,0,0; 0,0,-2];
for ui32Case = 1:3
    [bHit, dDistance, bFailure, dPoint, dJacOrigin, dJacAtt] = ...
        RayEllipsoidIntersection(dOrigins(:,ui32Case), dDirections(:,ui32Case), ...
            zeros(3,1), ones(3,1));
    testCase.verifyFalse(bHit);
    testCase.verifyEqual(bFailure, ui32Case ~= 1);
    testCase.verifyEqual(dDistance, 0);
    testCase.verifyEqual(dPoint, zeros(3,1));
    testCase.verifyEqual(dJacOrigin, zeros(1,3));
    testCase.verifyEqual(dJacAtt, zeros(1,3));
end
end

function testDerivativeUsesEstimateRatherThanReference(testCase)
dOrigin = [4; -1; 0.5];
dDirection = NormalizeVec_([-1; 0.2; -0.1]);
dCentre = [0.5; -0.25; 0.2];
dInvDiag = [1/9; 1/4; 1/1.44];
dEstimate = RotFromAxis_([2; 1; -1], 0.15);
dReference = RotFromAxis_([1; -2; 3], 0.4);
[~, dDistance, ~, ~, dJacOrigin, dJacAtt] = RayEllipsoidIntersection( ...
    dOrigin, dDirection, dCentre, dInvDiag, dReference, dEstimate);
[~, dDistanceCheck, ~, ~, dOriginCheck, dAttCheck] = RayEllipsoidIntersection( ...
    dOrigin, dDirection, dCentre, dInvDiag, eye(3), dEstimate);
testCase.verifyEqual(dDistanceCheck, dDistance);
testCase.verifyEqual(dOriginCheck, dJacOrigin);
testCase.verifyEqual(dAttCheck, dJacAtt);
end

%% Helpers
function dIntersectDist = SolveRayEllipsoidDistance_(dOrigin, dDirection, dCentre, dInvDiag)
% Regenerated by GPT 5.1 Codex for comparison
M = diag(dInvDiag);
o = dOrigin - dCentre;
a = dDirection' * M * dDirection;
b = dDirection' * M * o;
c = o' * M * o - 1;
delta = b^2 - a * c;

if delta < -eps || abs(a) < eps
    dIntersectDist = NaN;
    return
end

t0 = (-b + sqrt(max(delta, 0))) / a;
t1 = (-b - sqrt(max(delta, 0))) / a;
vals = [t0, t1];
vals = vals(vals >= -sqrt(eps));

if isempty(vals)
    dIntersectDist = NaN;
else
    dIntersectDist = min(vals);
    if dIntersectDist < 0
        dIntersectDist = 0;
    end
end
end

function dVec = NormalizeVec_(dVec)
dVec = dVec / norm(dVec);
end

function dRotation = RotFromAxis_(dAxis, dAngle)
% Express the axis-angle fixture as the shared utility's rotation-vector input.
dRotation = RotationVectorToDCM(NormalizeVec_(dAxis) * dAngle);
end

function dIntersectDist = EvalDistanceWithOrigin_(dOrigin, dDirection, dCentre, dInvDiag, R)
[bHit, d] = RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag, R, R, [false, false]);
assert(bHit, 'Finite-diff dOrigin perturbation lost intersection.');
dIntersectDist = d;
end

function dIntersectDist = EvalDistanceWithAttErr_(theta, dOrigin, dDirection, dCentre, dInvDiag, Rtrue)
Rpert = RotationVectorToDCM(theta) * Rtrue;
[bHit, d] = RayEllipsoidIntersection(dOrigin, dDirection, dCentre, dInvDiag, Rtrue, Rpert, [false, false]);
assert(bHit, 'Finite-diff attitude perturbation lost intersection.');
dIntersectDist = d;
end
