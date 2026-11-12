function [dSymmetricInertia_TB, dPseudoInertiaCholFactor] = ValidatePhysicalInertia(dInertia_TB) %#codegen
%% SIGNATURE
% [dSymmetricInertia_TB, dPseudoInertiaCholFactor] = ValidatePhysicalInertia(dInertia_TB)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Check that dInertia_TB is the inertia tensor of a physical rigid body about its center of mass, and return
% its symmetric part. The tensor must be finite and symmetric within 1e-12 of its Frobenius norm, and the
% pseudo-inertia Sigma = 0.5 * trace(J) * I - J of its symmetric part must be positive definite. Sigma is
% positive definite exactly when the principal moments are positive and satisfy the strict triangle
% inequalities. The lower Cholesky factor of Sigma is returned for callers that perturb the inertia in its
% log-Cholesky chart.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dInertia_TB                (3,3) double candidate center-of-mass inertia tensor [kg m^2]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dSymmetricInertia_TB       (3,3) double symmetric part of dInertia_TB [kg m^2]
% dPseudoInertiaCholFactor   (3,3) double lower Cholesky factor of the pseudo-inertia of dSymmetricInertia_TB
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-11-2026  Pietro Califano, Claude Opus 5.5   Merge the inertia checks of STargetRotationalStateConfig,
%                                                PropagateTorqueFreeBodyHistory and PerturbInertiaTensor.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dInertia_TB (3,3) double {mustBeReal}
end
arguments (Output)
    dSymmetricInertia_TB (3,3) double
    dPseudoInertiaCholFactor (3,3) double
end

% Require finite entries and symmetry within a tolerance relative to the tensor norm
if any(~isfinite(dInertia_TB), 'all')
    error('ValidatePhysicalInertia:InvalidInertiaTensor', ...
        'Inertia tensor must be finite.');
end

dSymmetryTolerance = 1.0e-12 * norm(dInertia_TB, 'fro');
if norm(dInertia_TB - transpose(dInertia_TB), 'fro') > dSymmetryTolerance
    error('ValidatePhysicalInertia:InvalidInertiaTensor', ...
        'Inertia tensor must be symmetric.');
end

% Require a positive definite pseudo-inertia of the symmetric part
dSymmetricInertia_TB = 0.5 * (dInertia_TB + transpose(dInertia_TB));
dPseudoInertia = 0.5 * trace(dSymmetricInertia_TB) * eye(3) - dSymmetricInertia_TB;
[dPseudoInertiaCholFactor, dCholFailure] = chol(dPseudoInertia, 'lower');
if dCholFailure ~= 0.0
    error('ValidatePhysicalInertia:InvalidInertiaTensor', ...
        'Inertia tensor must have positive principal moments that satisfy the strict triangle inequalities.');
end

end
