function dPerturbedInertia_TB = PerturbInertiaTensor(dNominalInertia_TB, ...
                                                     dInertiaChartError) %#codegen
%% SIGNATURE
% dPerturbedInertia_TB = PerturbInertiaTensor(dNominalInertia_TB, dInertiaChartError)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply one deterministic six-coordinate relative log-Cholesky perturbation to a physically consistent rigid-body
% inertia tensor. The map factors the pseudo-inertia covariance
%
%   Sigma = 0.5 * trace(J) * I - J
%
% and composes its nominal lower Cholesky factor with a dimensionless lower-triangular relative factor. Coordinates
% [1, 3, 6] are logarithmic diagonal errors and [2, 4, 5] are the lower off-diagonal errors in row order. Every
% numerically representable input maps directly to a symmetric positive-definite inertia satisfying all strict triangle
% inequalities; this routine performs no random sampling, eigenspace selection, rejection, or retry.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dNominalInertia_TB    (3,3) double nominal physical target-frame inertia tensor [kg m^2]
% dInertiaChartError    (6,1) double dimensionless relative log-Cholesky chart error
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPerturbedInertia_TB  (3,3) double perturbed physical target-frame inertia tensor [kg m^2]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-08-2026  Pietro Califano, Codex     Replace nondeterministic rejection-sampling prototype.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dNominalInertia_TB (3,3) double {mustBeReal, mustBeFinite}
    dInertiaChartError (6,1) double {mustBeReal, mustBeFinite}
end
arguments (Output)
    dPerturbedInertia_TB (3,3) double
end

% Require the nominal input itself to represent one physical inertia. The
% pseudo-inertia covariance is positive definite exactly when the inertia is
% positive definite and satisfies all strict triangle inequalities.
dSymmetryTolerance = 1.0e-12 * norm(dNominalInertia_TB, 'fro');
if norm(dNominalInertia_TB - transpose(dNominalInertia_TB), 'fro') > dSymmetryTolerance
    error('PerturbInertiaTensor:InvalidNominalInertia', ...
        'Nominal inertia must be finite, symmetric, and strictly physical.');
end
dNominalInertia_TB = 0.5 * (dNominalInertia_TB + transpose(dNominalInertia_TB));
dNominalPseudoInertia = 0.5 * trace(dNominalInertia_TB) * eye(3) - dNominalInertia_TB;
[dNominalCholFactor, dNominalCholFailure] = chol(dNominalPseudoInertia, 'lower');
if dNominalCholFailure ~= 0.0
    error('PerturbInertiaTensor:InvalidNominalInertia', ...
        'Nominal inertia must be finite, symmetric, and strictly physical.');
end

% Map the unconstrained six-vector to a dimensionless lower-triangular
% factor with positive diagonal, then compose it with the nominal factor.
dRelativeCholFactor = zeros(3);
dRelativeCholFactor(1, 1) = exp(dInertiaChartError(1));
dRelativeCholFactor(2, 1) = dInertiaChartError(2);
dRelativeCholFactor(2, 2) = exp(dInertiaChartError(3));
dRelativeCholFactor(3, 1) = dInertiaChartError(4);
dRelativeCholFactor(3, 2) = dInertiaChartError(5);
dRelativeCholFactor(3, 3) = exp(dInertiaChartError(6));
if any(~isfinite(dRelativeCholFactor), 'all') || any(diag(dRelativeCholFactor) <= 0.0)
    error('PerturbInertiaTensor:ChartOutOfRange', ...
        'Inertia chart error exceeds the numerically representable physical domain.');
end

% Reconstruct through the inverse pseudo-inertia map. Symmetrization removes
% roundoff asymmetry without altering the physical construction.
dPerturbedCholFactor = dNominalCholFactor * dRelativeCholFactor;
dPerturbedPseudoInertia = dPerturbedCholFactor * transpose(dPerturbedCholFactor);
dPerturbedInertia_TB = trace(dPerturbedPseudoInertia) * eye(3) - dPerturbedPseudoInertia;
dPerturbedInertia_TB = 0.5 * (dPerturbedInertia_TB + transpose(dPerturbedInertia_TB));

% Detect only floating-point range loss. This is not a stochastic
% admissibility gate: representable chart points are physical by construction.
[~, dPerturbedCholFailure] = chol(dPerturbedInertia_TB, 'lower');
if any(~isfinite(dPerturbedInertia_TB), 'all') || dPerturbedCholFailure ~= 0.0
    error('PerturbInertiaTensor:ChartOutOfRange', ...
        'Inertia chart error exceeds the numerically representable physical domain.');
end

end
