function dAttitudeCovariance_IN = PropagateTargetAttCovariance( ...
    dInitialAttCovariance_IN, dInitialAngVelCov_IN, ...
    dNominalAngVel_IN, dElapsedTime) %#codegen
%% SIGNATURE
% dAttitudeCovariance_IN = PropagateTargetAttCovariance(dInitialAttCovariance_IN, ...
%     dInitialAngVelCov_IN, dNominalAngVel_IN, dElapsedTime)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Propagate an inertial left target-attitude error covariance for the nominal model
% R(t) = Exp(-skew(omega_IN) * t) * R(0). The initial attitude error and one time-constant inertial angular-rate
% error are statistically independent. The runtime algorithm is analytic and does not use finite differences,
% sigma points, deterministic samples, Monte Carlo propagation, or covariance decompositions. Positive-definite
% initial uncertainty is an upstream configuration invariant; this operation retains only cheap numerical guards.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dInitialAttCovariance_IN   Initial inertial left attitude-error covariance [rad^2].
% dInitialAngVelCov_IN      Initial inertial angular-rate-error covariance [rad^2/s^2].
% dNominalAngVel_IN         Nominal constant angular velocity expressed in inertial coordinates [rad/s].
% dElapsedTime              Nonnegative propagation interval [s].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAttitudeCovariance_IN    Propagated inertial left attitude-error covariance [rad^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-08-2026  Pietro Califano, Codex gpt-5.6     First implementation.
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

% Integrate the linearized inertial left-error dynamics exactly. The left
% Jacobian is evaluated with a series near zero to avoid cancellation.
dIntegratedRotationVector = -dElapsedTime .* dNominalAngVel_IN;
dAttitudeTransition = RotationVectorToDCM(dIntegratedRotationVector);
dRateErrorMapping = -dElapsedTime .* ComputeLeftJacobian_(dIntegratedRotationVector);

dAttitudeCovariance_IN = dAttitudeTransition * dInitialAttCovariance_IN * ...
    transpose(dAttitudeTransition) + dRateErrorMapping * dInitialAngVelCov_IN * ...
    transpose(dRateErrorMapping);
dAttitudeCovariance_IN = 0.5 .* (dAttitudeCovariance_IN + ...
    transpose(dAttitudeCovariance_IN));
end

function dLeftJacobian = ComputeLeftJacobian_(dRotationVector) %#codegen
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
if any(not(isfinite(dCovariance)), 'all')
    error(charErrorID, '%s must be finite and symmetric.', charDescription);
end

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
