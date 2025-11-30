function tests = testRayEllipsoidIntersection
% Unit tests for RayEllipsoidIntersection covering sphere/ellipsoid cases,
% misses, tangency, rotations, and jacobian consistency.
tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(repoRoot);
SetupPaths_EstimationGears;
end

function testSphereFromOutside(testCase)
origin = [0; 0; 3];
direction = [0; 0; -1];
centre = zeros(3,1);
invDiag = ones(3,1);

[hit, dist, failure, point] = RayEllipsoidIntersection(origin, direction, centre, invDiag);
testCase.verifyTrue(hit);
testCase.verifyFalse(failure);
testCase.verifyEqual(dist, 2, 'AbsTol', 1e-12);
testCase.verifyEqual(point, [0; 0; 1], 'AbsTol', 1e-12);
end

function testSphereFromInside(testCase)
origin = [0; 0; 0];
direction = [1; 0; 0];
centre = zeros(3,1);
invDiag = ones(3,1);

[hit, dist, failure] = RayEllipsoidIntersection(origin, direction, centre, invDiag);
testCase.verifyTrue(hit);
testCase.verifyFalse(failure);
testCase.verifyEqual(dist, 1, 'AbsTol', 1e-12);
end

function testMissedIntersection(testCase)
origin = [0; 0; 3];
direction = [0; 0; 1]; % Pointing away from sphere
centre = zeros(3,1);
invDiag = ones(3,1);

[hit, dist, failure] = RayEllipsoidIntersection(origin, direction, centre, invDiag);
testCase.verifyFalse(hit);
testCase.verifyFalse(failure);
testCase.verifyEqual(dist, 0, 'AbsTol', 1e-12);
end

function testTangentialContactSetsFailure(testCase)
origin = [1; 0; 0];
direction = [0; 1; 0]; % Tangent on unit sphere
centre = zeros(3,1);
invDiag = ones(3,1);

[hit, dist, failure] = RayEllipsoidIntersection(origin, direction, centre, invDiag);
testCase.verifyTrue(hit);
testCase.verifyTrue(failure); % Tangency -> ill-conditioned jacobians
testCase.verifyEqual(dist, 0, 'AbsTol', 1e-12);
end

function testRotatedEllipsoidMatchesAnalytic(testCase)
origin = [2; -0.5; 1.2];
direction = normalizeVec([-1; 0.1; -0.2]);
centre = [0.3; -0.1; 0.2];
invDiag = [1/9; 1/4; 1/2.25];
R = rotFromAxis([0; 0; 1], deg2rad(20)) * rotFromAxis([0; 1; 0], deg2rad(15));

[hit, dist, failure, point] = RayEllipsoidIntersection(origin, direction, centre, invDiag, R, R);
testCase.verifyTrue(hit);
testCase.verifyFalse(failure);

expectedDist = solveRayEllipsoidDistance(R * origin, R * direction, R * centre, invDiag);
testCase.verifyEqual(dist, expectedDist, 'AbsTol', 1e-10);
testCase.verifyEqual(point, origin + direction * dist, 'AbsTol', 1e-10);
end

function testJacobiansAgainstFiniteDiff(testCase)
origin = [4; -1; 0.5];
direction = normalizeVec([-1; 0.2; -0.1]);
centre = [0.5; -0.25; 0.2];
invDiag = [1/9; 1/4; 1/1.44];
Rtrue = rotFromAxis([0; 0; 1], deg2rad(25));

[hit, dist, failure, ~, jacOrigin, jacAtt] = RayEllipsoidIntersection(origin, direction, centre, invDiag, Rtrue, Rtrue);
testCase.verifyTrue(hit);
testCase.verifyFalse(failure);
testCase.verifyGreaterThan(dist, 0);

fdJacOrigin = finiteDiff(@(o) evalDistanceWithOrigin(o, direction, centre, invDiag, Rtrue), origin, 1e-6);
testCase.verifyEqual(jacOrigin, fdJacOrigin, 'AbsTol', 5e-6);

fdJacAtt = finiteDiff(@(theta) evalDistanceWithAttErr(theta, origin, direction, centre, invDiag, Rtrue), zeros(3,1), 1e-6);
testCase.verifyEqual(jacAtt, fdJacAtt, 'AbsTol', 5e-6);
end

%% Helpers
function dist = solveRayEllipsoidDistance(origin, direction, centre, invDiag)
M = diag(invDiag);
o = origin - centre;
a = direction' * M * direction;
b = direction' * M * o;
c = o' * M * o - 1;
delta = b^2 - a * c;

if delta < -eps || abs(a) < eps
    dist = NaN;
    return
end

t0 = (-b + sqrt(max(delta, 0))) / a;
t1 = (-b - sqrt(max(delta, 0))) / a;
vals = [t0, t1];
vals = vals(vals >= -sqrt(eps));
if isempty(vals)
    dist = NaN;
else
    dist = min(vals);
    if dist < 0
        dist = 0;
    end
end
end

function v = normalizeVec(v)
v = v / norm(v);
end

function R = rotFromAxis(axis, angle)
ax = normalizeVec(axis);
K = [  0,   -ax(3),  ax(2);
      ax(3),   0,   -ax(1);
     -ax(2), ax(1),   0  ];
R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);
end

function jac = finiteDiff(funHandle, x0, step)
n = numel(x0);
jac = zeros(1, n);
for idx = 1:n
    dx = zeros(size(x0));
    dx(idx) = step;
    fp = funHandle(x0 + dx);
    fm = funHandle(x0 - dx);
    jac(idx) = (fp - fm) / (2 * step);
end
end

function dist = evalDistanceWithOrigin(origin, direction, centre, invDiag, R)
[hit, d] = RayEllipsoidIntersection(origin, direction, centre, invDiag, R, R, [false, false]);
assert(hit, 'Finite-diff origin perturbation lost intersection.');
dist = d;
end

function dist = evalDistanceWithAttErr(theta, origin, direction, centre, invDiag, Rtrue)
Rpert = smallRotation(theta) * Rtrue;
[hit, d] = RayEllipsoidIntersection(origin, direction, centre, invDiag, Rtrue, Rpert, [false, false]);
assert(hit, 'Finite-diff attitude perturbation lost intersection.');
dist = d;
end

function R = smallRotation(theta)
angle = norm(theta);
if angle < eps
    R = eye(3);
    return
end
axis = theta / angle;
R = rotFromAxis(axis, angle);
end
