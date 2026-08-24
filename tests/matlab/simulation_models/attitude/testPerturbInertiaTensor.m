function tests = testPerturbInertiaTensor
%% SIGNATURE
% tests = testPerturbInertiaTensor
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify the deterministic relative log-Cholesky perturbation map for physically consistent rigid-body inertia.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB unit-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% PerturbInertiaTensor.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function testZeroErrorRoundTripsWithoutConsumingRng(testCase)
dNominalInertia_TB = [4.0, 0.3, -0.2; ...
                      0.3, 5.0, 0.4; ...
                     -0.2, 0.4, 6.0];
strRngBefore = rng;

dPerturbedInertia_TB = PerturbInertiaTensor(dNominalInertia_TB, zeros(6, 1));

strRngAfter = rng;
verifyEqual(testCase, dPerturbedInertia_TB, dNominalInertia_TB, ...
    'AbsTol', 5.0e-14);
verifyEqual(testCase, strRngAfter, strRngBefore);
end

function testMatchesKnownRelativeCholeskyChartSample(testCase)
% The nominal pseudo-inertia covariance is diag([3, 2, 1]). The chosen
% chart sample has exact triangular factors, providing an oracle independent
% of the implementation under test.
dNominalInertia_TB = diag([3.0, 4.0, 5.0]);
dInertiaChartError = [log(2.0); 0.5; log(3.0); ...
                      -0.25; 0.75; log(0.5)];
dExpectedInertia_TB = [19.375, -sqrt(6.0), 0.5 * sqrt(3.0); ...
                       -sqrt(6.0), 12.875, -(17.0 / 8.0) * sqrt(2.0); ...
                       0.5 * sqrt(3.0), -(17.0 / 8.0) * sqrt(2.0), 30.5];

dPerturbedInertia_TB = PerturbInertiaTensor(dNominalInertia_TB, dInertiaChartError);

verifyEqual(testCase, dPerturbedInertia_TB, dExpectedInertia_TB, ...
    'AbsTol', 2.0e-13);
end

function testRepeatedMomentsAndLargeErrorsRemainPhysical(testCase)
dNominalInertia_TB = 2.0 * eye(3);
dInertiaChartError = [2.0; -1.5; -1.0; 0.75; -2.0; 1.0];

dPerturbedInertia_TB = PerturbInertiaTensor(dNominalInertia_TB, dInertiaChartError);

dPrincipalMoments = eig(dPerturbedInertia_TB);
dPseudoInertiaCovariance = 0.5 * trace(dPerturbedInertia_TB) * eye(3) - dPerturbedInertia_TB;
[~, dCholFailure] = chol(dPseudoInertiaCovariance, 'lower');
verifyEqual(testCase, dPerturbedInertia_TB, transpose(dPerturbedInertia_TB), 'AbsTol', 2.0e-13);
verifyGreaterThan(testCase, min(dPrincipalMoments), 0.0);
verifyTrue(testCase, all(dPrincipalMoments < ...
    sum(dPrincipalMoments) - dPrincipalMoments));
verifyEqual(testCase, dCholFailure, 0.0);
end

function testRejectsNonphysicalNominalInertia(testCase)
dNonphysicalInertia_TB = diag([1.0, 1.0, 3.0]);

verifyError(testCase, @() PerturbInertiaTensor(dNonphysicalInertia_TB, zeros(6, 1)), ...
    'PerturbInertiaTensor:InvalidNominalInertia');
end

function testRejectsAsymmetryRelativeToSmallInertiaScale(testCase)
% Symmetry is a physical tensor contract and must be evaluated relative to
% the inertia scale rather than an absolute unit-sized tolerance.
dAsymmetricInertia_TB = [3.0e-10, 1.0e-13, 0.0; ...
                         0.0,     4.0e-10, 0.0; ...
                         0.0,     0.0,     5.0e-10];

verifyError(testCase, @() PerturbInertiaTensor(dAsymmetricInertia_TB, zeros(6, 1)), ...
    'PerturbInertiaTensor:InvalidNominalInertia');
end
