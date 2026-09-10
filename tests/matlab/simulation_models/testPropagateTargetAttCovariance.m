function tests = testPropagateTargetAttCovariance
%% SIGNATURE
% tests = testPropagateTargetAttCovariance
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify frame-consistent analytic propagation of inertial left target-attitude uncertainty under a constant
% inertial angular velocity and one time-constant angular-rate error.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tests    MATLAB unit-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-08-2026  Pietro Califano, Codex gpt-5.6     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% PropagateTargetAttCovariance, RotationVectorToDCM, LogMap_SO3toR3, skewSymm.
% -------------------------------------------------------------------------------------------------------------

tests = functiontests(localfunctions);
end

function testMatchesAugmentedStateTransition(testCase)
dInitialAttCovariance_IN = [4.0e-6, 0.7e-6, -0.3e-6; ...
                            0.7e-6, 9.0e-6,  0.5e-6; ...
                           -0.3e-6, 0.5e-6, 16.0e-6];
dInitialAngVelCov_IN = [2.5e-13, -0.4e-13, 0.2e-13; ...
                       -0.4e-13,  4.0e-13, 0.3e-13; ...
                        0.2e-13,  0.3e-13, 6.0e-13];
dNominalAngVel_IN = [1.7e-4; -0.8e-4; 2.2e-4];
dElapsedTime = 2735.0;

dAugmentedDynamics = [-skewSymm(dNominalAngVel_IN), -eye(3); zeros(3), zeros(3)];
dAugmentedTransition = expm(dAugmentedDynamics .* dElapsedTime);
dInitialAugmentedCovariance = blkdiag(dInitialAttCovariance_IN, dInitialAngVelCov_IN);
dExpectedAugmentedCovariance = dAugmentedTransition * dInitialAugmentedCovariance * ...
    transpose(dAugmentedTransition);
dExpectedAttitudeCovariance_IN = dExpectedAugmentedCovariance(1:3, 1:3);

dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, dNominalAngVel_IN, dElapsedTime);

verifyEqual(testCase, dAttitudeCovariance_IN, dExpectedAttitudeCovariance_IN, ...
    'RelTol', 2.0e-12, 'AbsTol', 2.0e-18);
end

function testZeroSpinRecoversAdditiveLimit(testCase)
dInitialAttCovariance_IN = diag([2.0e-3, 3.0e-3, 5.0e-3].^2);
dInitialAngVelCov_IN = diag([2.0e-7, 4.0e-7, 7.0e-7].^2);
dElapsedTime = 740.0;
dExpectedCovariance_IN = dInitialAttCovariance_IN + ...
    dElapsedTime.^2 .* dInitialAngVelCov_IN;

dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, zeros(3,1), dElapsedTime);

verifyEqual(testCase, dAttitudeCovariance_IN, dExpectedCovariance_IN, 'AbsTol', 2.0e-18);
end

function testZeroTimePreservesInitialCovariance(testCase)
dInitialAttCovariance_IN = diag([2.0e-3, 3.0e-3, 5.0e-3].^2);
dInitialAngVelCov_IN = diag([2.0e-7, 4.0e-7, 7.0e-7].^2);

dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, [1.0e-4; -2.0e-4; 3.0e-4], 0.0);

verifyEqual(testCase, dAttitudeCovariance_IN, dInitialAttCovariance_IN, 'AbsTol', 0.0);
end

function testSmallAccumulatedRotationMatchesStateTransition(testCase)
dInitialAttCovariance_IN = diag([2.0e-3, 3.0e-3, 5.0e-3].^2);
dInitialAngVelCov_IN = diag([2.0e-7, 4.0e-7, 7.0e-7].^2);
dNominalAngVel_IN = [1.0e-10; -2.0e-10; 3.0e-10];
dElapsedTime = 0.25;
dAugmentedDynamics = [-skewSymm(dNominalAngVel_IN), -eye(3); zeros(3), zeros(3)];
dAugmentedTransition = expm(dAugmentedDynamics .* dElapsedTime);
dExpectedCovariance = dAugmentedTransition * blkdiag(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN) * transpose(dAugmentedTransition);

dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, dNominalAngVel_IN, dElapsedTime);

verifyEqual(testCase, dAttitudeCovariance_IN, dExpectedCovariance(1:3,1:3), 'AbsTol', 2.0e-18);
end

function testRotatedInertialCoordinatesRemainEquivalent(testCase)
dCoordinateRotation = RotationVectorToDCM([0.31; -0.22; 0.17]);
dInitialAttCovariance_IN = diag([2.0e-3, 4.0e-3, 7.0e-3].^2);
dInitialAngVelCov_IN = diag([1.0e-7, 3.0e-7, 6.0e-7].^2);
dNominalAngVel_IN = [0.6e-4; -1.3e-4; 2.1e-4];
dElapsedTime = 1860.0;

dReferenceCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, dNominalAngVel_IN, dElapsedTime);
dRotatedCovariance_IN = PropagateTargetAttCovariance( ...
    dCoordinateRotation * dInitialAttCovariance_IN * transpose(dCoordinateRotation), ...
    dCoordinateRotation * dInitialAngVelCov_IN * transpose(dCoordinateRotation), ...
    dCoordinateRotation * dNominalAngVel_IN, dElapsedTime);

verifyEqual(testCase, dRotatedCovariance_IN, ...
    dCoordinateRotation * dReferenceCovariance_IN * transpose(dCoordinateRotation), ...
    'RelTol', 2.0e-12, 'AbsTol', 2.0e-18);
end

function testLongMissionIntervalMatchesStateTransition(testCase)
dInitialAttCovariance_IN = [2.5e-7, 0.3e-7, -0.1e-7; ...
                            0.3e-7, 2.8e-7,  0.2e-7; ...
                           -0.1e-7, 0.2e-7,  3.1e-7];
dInitialAngVelCov_IN = [6.9e-13, 0.4e-13, -0.2e-13; ...
                        0.4e-13, 7.2e-13,  0.3e-13; ...
                       -0.2e-13, 0.3e-13,  7.5e-13];
dNominalAngVel_IN = [0.2; -0.1; 1.0];
dNominalAngVel_IN = dNominalAngVel_IN ./ norm(dNominalAngVel_IN) .* (2.0 .* pi ./ (12.1324 .* 3600.0));
dElapsedTime = 337200.0;
dAugmentedDynamics = [-skewSymm(dNominalAngVel_IN), -eye(3); zeros(3), zeros(3)];
dAugmentedTransition = expm(dElapsedTime .* dAugmentedDynamics);
dExpectedCovariance = dAugmentedTransition * blkdiag(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN) * transpose(dAugmentedTransition);

dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, dNominalAngVel_IN, dElapsedTime);

verifyEqual(testCase, dAttitudeCovariance_IN, dExpectedCovariance(1:3,1:3), ...
    'RelTol', 5.0e-12, 'AbsTol', 2.0e-16);
end

function testNonlinearRotationSamplesValidateLocalModel(testCase)
dInitialAttCovariance_IN = [4.0e-6, 0.7e-6, -0.3e-6; ...
                            0.7e-6, 9.0e-6,  0.5e-6; ...
                           -0.3e-6, 0.5e-6, 16.0e-6];
dInitialAngVelCov_IN = [2.5e-13, -0.4e-13, 0.2e-13; ...
                       -0.4e-13,  4.0e-13, 0.3e-13; ...
                        0.2e-13,  0.3e-13, 6.0e-13];
dNominalAngVel_IN = [1.7e-4; -0.8e-4; 2.2e-4];
dElapsedTime = 1800.0;
ui32SampleCount = uint32(50000);
objRandomStream = RandStream('mt19937ar', 'Seed', 24680);
dInitialAttCovSqrtLower = chol(dInitialAttCovariance_IN, 'lower');
dInitialAngVelCovSqrtLower = chol(dInitialAngVelCov_IN, 'lower');
dNominalDCM_INfromTB = RotationVectorToDCM(-dElapsedTime .* dNominalAngVel_IN);
dPropagatedLeftErrors_IN = zeros(3, ui32SampleCount);

% Propagate sampled rotations through the nonlinear constant-spin model so
% this test validates the linearized marginal against the physical model.
for ui32SampleIdx = uint32(1):ui32SampleCount
    dInitialAttitudeError_IN = dInitialAttCovSqrtLower * randn(objRandomStream, 3, 1);
    dAngularVelocityError_IN = dInitialAngVelCovSqrtLower * randn(objRandomStream, 3, 1);
    dTrueDCM_INfromTB = RotationVectorToDCM(-dElapsedTime .* ...
        (dNominalAngVel_IN + dAngularVelocityError_IN)) * RotationVectorToDCM(dInitialAttitudeError_IN);
    dPropagatedLeftErrors_IN(:,ui32SampleIdx) = LogMap_SO3toR3(dTrueDCM_INfromTB * ...
        transpose(dNominalDCM_INfromTB));
end

dSampleCovariance_IN = cov(transpose(dPropagatedLeftErrors_IN), 1);
dExpectedCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, dNominalAngVel_IN, dElapsedTime);
dRelativeSamplingError = norm(dSampleCovariance_IN - dExpectedCovariance_IN, 'fro') ./ ...
    norm(dExpectedCovariance_IN, 'fro');

verifyLessThan(testCase, dRelativeSamplingError, 0.03);
end

function testRejectsMalformedCovariance(testCase)
dAsymmetricCovariance = eye(3);
dAsymmetricCovariance(1,2) = 0.2;
dNonfiniteCovariance = eye(3);
dNonfiniteCovariance(1,1) = NaN;

verifyError(testCase, @() PropagateTargetAttCovariance(dAsymmetricCovariance, ...
    eye(3), zeros(3,1), 1.0), 'PropagateTargetAttCovariance:InvalidAttCovariance');
verifyError(testCase, @() PropagateTargetAttCovariance(eye(3), dNonfiniteCovariance, ...
    zeros(3,1), 1.0), ...
    'PropagateTargetAttCovariance:InvalidAngVelCovariance');
end
