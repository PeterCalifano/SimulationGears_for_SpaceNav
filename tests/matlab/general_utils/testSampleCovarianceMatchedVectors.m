function tests = testSampleCovarianceMatchedVectors
%% SIGNATURE
% tests = testSampleCovarianceMatchedVectors
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify deterministic caller-stream covariance sampling for Gaussian and covariance-matched uniform solid-ellipsoid
% distributions.
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
% EnumCovarianceSamplingDistribution, SampleCovarianceMatchedVectors.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function testGaussianMatchesCallerStreamReference(testCase)
dCovarianceFactor = [0.4, 0.0, 0.0; ...
                     0.1, 0.2, 0.0; ...
                    -0.2, 0.3, 0.5];
dCovariance = dCovarianceFactor * transpose(dCovarianceFactor);
ui32NumSamples = uint32(7);
objReferenceStream = RandStream('mt19937ar', 'Seed', 2468);
objActualStream = RandStream('mt19937ar', 'Seed', 2468);
dExpectedSamples = dCovarianceFactor * randn(objReferenceStream, 3, double(ui32NumSamples));
strGlobalRngBefore = rng;

dActualSamples = SampleCovarianceMatchedVectors(objActualStream, dCovariance, ui32NumSamples, ...
    EnumCovarianceSamplingDistribution.GAUSSIAN);

verifyEqual(testCase, dActualSamples, dExpectedSamples, 'AbsTol', 1.0e-15);
verifyEqual(testCase, rng, strGlobalRngBefore);
end

function testUniformSolidEllipsoidMatchesConfiguredCovariance(testCase)
dCovarianceFactor = [0.4, 0.0, 0.0; ...
                     0.1, 0.2, 0.0; ...
                    -0.2, 0.3, 0.5];
dCovariance = dCovarianceFactor * transpose(dCovarianceFactor);
ui32NumSamples = uint32(20000);
objRandomStream = RandStream('mt19937ar', 'Seed', 13579);

dSamples = SampleCovarianceMatchedVectors(objRandomStream, dCovariance, ui32NumSamples, ...
    EnumCovarianceSamplingDistribution.UNIFORM_SOLID_ELLIPSOID);

dWhitenedSamples = dCovarianceFactor \ dSamples;
dSquaredRadii = sum(dWhitenedSamples.^2, 1);
dEmpiricalCovariance = cov(transpose(dSamples), 1);
verifyLessThanOrEqual(testCase, max(dSquaredRadii), 5.0 * (1.0 + 1.0e-12));
verifyLessThan(testCase, ...
    norm(dEmpiricalCovariance - dCovariance, 'fro'), ...
    0.05 * norm(dCovariance, 'fro'));
end

function testZeroSamplesValidateWithoutAdvancingStream(testCase)
objRandomStream = RandStream('mt19937ar', 'Seed', 8642);
objReferenceStream = RandStream('mt19937ar', 'Seed', 8642);
dSingularCovariance = diag([1.0, 0.0, 0.25]);

dSamples = SampleCovarianceMatchedVectors(objRandomStream, dSingularCovariance, uint32(0), ...
    EnumCovarianceSamplingDistribution.GAUSSIAN);

verifySize(testCase, dSamples, [3, 0]);
verifyEqual(testCase, rand(objRandomStream, 1, 4), ...
                      rand(objReferenceStream, 1, 4));
end

function testEnumPreservesCosmicaConfigurationToken(testCase)
enumDistribution = EnumCovarianceSamplingDistribution.FromConfigValue('uniform_ellipsoid');

verifyEqual(testCase, enumDistribution, ...
    EnumCovarianceSamplingDistribution.UNIFORM_SOLID_ELLIPSOID);
verifyEqual(testCase, enumDistribution.ToConfigValue(), ...
    'uniform_ellipsoid');
verifyEqual(testCase, ...
    EnumCovarianceSamplingDistribution.FromConfigValue('uniform_solid_ellipsoid'), ...
    enumDistribution);
end

function testRejectsMalformedCovariance(testCase)
objRandomStream = RandStream('mt19937ar', 'Seed', 1234);

verifyError(testCase, @() SampleCovarianceMatchedVectors(objRandomStream, ...
    [1.0, 0.5; 0.0, 1.0], uint32(0), EnumCovarianceSamplingDistribution.GAUSSIAN), ...
    'SampleCovarianceMatchedVectors:NonSymmetricCovariance');
verifyError(testCase, @() SampleCovarianceMatchedVectors(objRandomStream, ...
    diag([1.0, -0.1]), uint32(0), EnumCovarianceSamplingDistribution.GAUSSIAN), ...
    'SampleCovarianceMatchedVectors:NonPositiveSemidefiniteCovariance');
end

function testRejectsAsymmetryRelativeToSmallCovarianceScale(testCase)
% An absolute tolerance anchored to one would silently symmetrize this
% covariance even though its skew part dominates the configured scale.
objRandomStream = RandStream('mt19937ar', 'Seed', 1234);
dAsymmetricCovariance = [1.0e-16, 1.0e-15; ...
                         0.0,     1.0e-16];

verifyError(testCase, @() SampleCovarianceMatchedVectors(objRandomStream, ...
    dAsymmetricCovariance, uint32(0), EnumCovarianceSamplingDistribution.GAUSSIAN), ...
    'SampleCovarianceMatchedVectors:NonSymmetricCovariance');
end

function testRejectsNegativeEigenvalueRelativeToSmallScale(testCase)
% A negative variance remains invalid even when every covariance entry is
% far below one in absolute units.
objRandomStream = RandStream('mt19937ar', 'Seed', 1234);
dIndefiniteCovariance = diag([1.0e-16, -1.0e-18]);

verifyError(testCase, @() SampleCovarianceMatchedVectors(objRandomStream, ...
    dIndefiniteCovariance, uint32(0), EnumCovarianceSamplingDistribution.GAUSSIAN), ...
    'SampleCovarianceMatchedVectors:NonPositiveSemidefiniteCovariance');
end
