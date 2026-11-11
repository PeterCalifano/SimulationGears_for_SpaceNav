function dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
    dInitialAngVelCov_IN, ...
    dNominalAngVel_IN, ...
    dElapsedTime) %#codegen
%% SIGNATURE
% dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
%     dInitialAngVelCov_IN, dNominalAngVel_IN, dElapsedTime)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Propagate an inertial left target-attitude error covariance for the nominal model
% R(t) = Exp(skew(omega_IN) * t) * R(0). The initial attitude error and one time-constant inertial angular-rate
% error are statistically independent. The runtime algorithm is analytic and does not use finite differences,
% sigma points, deterministic samples, Monte Carlo propagation, or covariance decompositions. Positive-definite
% initial uncertainty is an upstream configuration invariant; this operation retains only cheap numerical guards.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dInitialAttCovariance_IN   Initial inertial left attitude-error covariance [rad^2].
% dInitialAngVelCov_IN       Initial inertial angular-rate-error covariance [rad^2/s^2].
% dNominalAngVel_IN          Nominal constant angular velocity expressed in inertial coordinates [rad/s].
% dElapsedTime               Nonnegative propagation interval [s].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAttitudeCovariance_IN     Propagated inertial left attitude-error covariance [rad^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-08-2026  Pietro Califano, Codex gpt-5.6     First implementation.
% 23-08-2026  Pietro Califano, Codex             Correct the active body-to-inertial rotation sign.
% 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RotationVectorToDCM, skewSymm.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dInitialAttCovariance_IN (3,3) double
    dInitialAngVelCov_IN (3,3) double
    dNominalAngVel_IN (3,1) double
    dElapsedTime (1,1) double
end
arguments (Output)
    dAttitudeCovariance_IN (3,3) double
end

% Reject non-finite or asymmetric covariances, a non-finite rate and a negative interval
ValidateCovariance_(dInitialAttCovariance_IN, ...
    'PropagateTargetAttCovariance:InvalidAttCovariance', ...
    'Initial attitude covariance');
ValidateCovariance_(dInitialAngVelCov_IN, ...
    'PropagateTargetAttCovariance:InvalidAngVelCovariance', ...
    'Initial angular-rate covariance');
if any(not(isfinite(dNominalAngVel_IN)))
    error('PropagateTargetAttCovariance:InvalidAngularVelocity', ...
        'Nominal inertial angular velocity must be finite.');
elseif not(isfinite(dElapsedTime)) || dElapsedTime < 0.0
    error('PropagateTargetAttCovariance:InvalidElapsedTime', ...
        'Elapsed time must be finite and nonnegative.');
end

% Integrate the linearized inertial left-error dynamics exactly. The left Jacobian is evaluated with a series
% near zero to avoid cancellation.
dIntegratedRotationVector = dElapsedTime .* dNominalAngVel_IN;
dAttitudeTransition = RotationVectorToDCM(dIntegratedRotationVector);
dRateErrorMapping = dElapsedTime .* ComputeLeftJacobian_(dIntegratedRotationVector);

% Sum the attitude and rate-error contributions, then symmetrize the result
dAttitudeCovariance_IN = dAttitudeTransition * dInitialAttCovariance_IN * transpose(dAttitudeTransition) + ...
    dRateErrorMapping * dInitialAngVelCov_IN * transpose(dRateErrorMapping);
dAttitudeCovariance_IN = 0.5 .* (dAttitudeCovariance_IN + transpose(dAttitudeCovariance_IN));
end

function dLeftJacobian = ComputeLeftJacobian_(dRotationVector) %#codegen
%% SIGNATURE
% dLeftJacobian = ComputeLeftJacobian_(dRotationVector)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the left Jacobian of SO(3) at a rotation vector.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRotationVector   Rotation vector [rad].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dLeftJacobian     3x3 left Jacobian.
% -------------------------------------------------------------------------------------------------------------
dRotationMagnitude = norm(dRotationVector);
dSkewRotationVector = skewSymm(dRotationVector);
dSkewRotationVectorSquared = dSkewRotationVector * dSkewRotationVector;

if dRotationMagnitude < 1.0e-4
    % Use a series expansion to avoid cancellation for small angles.
    dMagnitudeSquared = dRotationMagnitude.^2;

    dFirstCoefficient = 0.5 - dMagnitudeSquared ./ 24.0 + dMagnitudeSquared.^2 ./ 720.0;
    dSecondCoefficient = 1.0 ./ 6.0 - dMagnitudeSquared ./ 120.0 + dMagnitudeSquared.^2 ./ 5040.0;
else
    % Use the exact formula for larger angles (Rodrigues' formula).
    dFirstCoefficient = (1.0 - cos(dRotationMagnitude)) ./ dRotationMagnitude.^2;
    dSecondCoefficient = (dRotationMagnitude - sin(dRotationMagnitude)) ./ dRotationMagnitude.^3;
end

dLeftJacobian = eye(3) + dFirstCoefficient .* dSkewRotationVector + ...
    dSecondCoefficient .* dSkewRotationVectorSquared;

end

function ValidateCovariance_(dCovariance, charErrorID, charDescription) %#codegen
%% SIGNATURE
% ValidateCovariance_(dCovariance, charErrorID, charDescription)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Raise charErrorID when the covariance holds a non-finite entry or is not symmetric within a tolerance
% relative to its Frobenius norm.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dCovariance       3x3 covariance to check.
% charErrorID       Error identifier to raise.
% charDescription   Name of the covariance in the error message.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------
if any(not(isfinite(dCovariance)), 'all')
    error(charErrorID, '%s must be finite and symmetric.', charDescription);
end

% Scale the symmetry tolerance by the covariance norm, floored at eps
dCovarianceScale = norm(dCovariance, 'fro');

if dCovarianceScale < eps
    dCovarianceScale = eps;
end

dSymmetryTolerance = 1.0e-12 .* dCovarianceScale;
dCovarianceAntisymmetric = dCovariance - transpose(dCovariance);
dSymmetryErrorSquared = sum(dCovarianceAntisymmetric.^2, 'all');

if dSymmetryErrorSquared > dSymmetryTolerance * dSymmetryTolerance
    error(charErrorID, '%s must be finite and symmetric.', charDescription);
end
end
