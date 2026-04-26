function [dPotentialPert, dAccPert_TB] = EvalExtSphHarmExpInTargetFrame( ...
    dPosSC_TB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
arguments
    dPosSC_TB           (3,1) double {mustBeFinite, mustBeReal}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeFinite, mustBeReal}
    dGravParam          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
%% PROTOTYPE
% [dPotentialPert, dAccPert_TB] = EvalExtSphHarmExpInTargetFrame( ...
%     dPosSC_TB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Canonical target-frame evaluator for the Exterior Spherical Harmonics
% Expansion model.
%
% This function returns the non-spherical potential and non-spherical
% acceleration in the target-body fixed frame using the repo-native
% unnormalized [Clm, Slm] storage convention.
%
% ACHTUNG: This function returns only the non-spherical contribution. The
% central-force term must be added separately if the total gravity field is
% required.
%
% Pole handling is treated explicitly when the in-plane radius tends to
% zero, avoiding the singular chain-rule form previously used by the
% legacy path.
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_TB:        [3x1]     Position in the target-body fixed frame.
% ui32MaxDegree:    [1]       Maximum harmonic degree.
% dCSlmCoeffCols:   [Nl x 2]  Unnormalized [Clm, Slm] coefficient table.
% dGravParam:       [1]       Gravitational parameter [LU^3/TU^2].
% dBodyRadiusRef:   [1]       Reference radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialPert:   [1]       Non-spherical potential [LU^2/TU^2].
% dAccPert_TB:      [3x1]     Non-spherical acceleration in the target-body
%                             fixed frame [LU/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 26-04-2026    Pietro Califano     Reuse shared spherical-to-Cartesian gradient mapping.
% 23-04-2026    Pietro Califano     Add canonical target-frame SHE evaluator.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalExtSphericalHarmExpCore()
% MapSphericalGradientToCartesian()
% -------------------------------------------------------------------------------------------------------------

%% Function code

% Input checks
dPosSCnorm = norm(dPosSC_TB);
if dPosSCnorm <= 0.0
    error('EvalExtSphHarmExpInTargetFrame:ZeroPosition', ...
        'The position norm must be strictly positive.');
end

% Compute geocentric latitude and longitude from the target-body-fixed position
dSCLat = asin(dPosSC_TB(3) / dPosSCnorm);
dSCLong = atan2(dPosSC_TB(2), dPosSC_TB(1));

% Evaluate the potential and its spherical-coordinate gradient using the shared SHE core
[dPotentialPert, dGradU] = EvalExtSphericalHarmExpCore( ...
    dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, ...
    dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

dAccPert_TB = MapSphericalGradientToCartesian(dPosSC_TB, dGradU);

end
