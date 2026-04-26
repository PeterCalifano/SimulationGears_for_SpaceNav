function [dPotentialPert, dGradU, dPlm] = EvalExtSphericalHarmExpCore( ...
    dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, dCSlmCoeffCols, ...
    dGravParam, dBodyRadiusRef) %#codegen
arguments
    dPosSCnorm          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dSCLat              (1,1) double {mustBeFinite, mustBeReal}
    dSCLong             (1,1) double {mustBeFinite, mustBeReal}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeFinite, mustBeReal}
    dGravParam          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
%% PROTOTYPE
% [dPotentialPert, dGradU, dPlm] = EvalExtSphericalHarmExpCore( ...
%     dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, dCSlmCoeffCols, ...
%     dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Shared core for the Exterior Spherical Harmonics Expansion model.
%
% This function evaluates the non-spherical potential together with its
% spherical-coordinate gradient at a target-body-fixed position specified
% through radius, latitude, and longitude.
%
% ACHTUNG: Coefficients must be UN-NORMALIZED and stored as [Clm, Slm]
% column pairs starting from (l,m) = (1,1). Degree-1 storage is accepted
% for format compatibility but ignored by the model, which starts at
% degree 2.
%
% This function uses BuildExtSphHarmExpBasis(), the same basis construction
% used by the field-to-coefficients least-squares fitter. MATLAB, codegen,
% evaluation, and fitting paths therefore share the same harmonic term math.
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCnorm:        [1]       Position norm in the target-body fixed frame.
% dSCLat:            [1]       Geocentric latitude [rad].
% dSCLong:           [1]       Longitude [rad].
% ui32MaxDegree:     [1]       Maximum harmonic degree.
% dCSlmCoeffCols:    [Nl x 2]  Unnormalized [Clm, Slm] coefficient table.
% dGravParam:        [1]       Gravitational parameter [LU^3/TU^2].
% dBodyRadiusRef:    [1]       Reference radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialPert:    [1]       Non-spherical potential [LU^2/TU^2].
% dGradU:            [3x1]     Gradient in spherical coordinates
%                              [dU/dr; dU/dLat; dU/dLong].
% dPlm:              [N x 1]   Flattened Legendre lookup table.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 26-04-2026    Pietro Califano     Reuse shared ExtSphHarm basis construction.
% 23-04-2026    Pietro Califano     Add shared SHE core for potential and gradient.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildExtSphHarmExpBasis()
% -------------------------------------------------------------------------------------------------------------

ui32RequiredRows = (ui32MaxDegree + uint32(1)) * (ui32MaxDegree + uint32(2)) / uint32(2) - uint32(2);
if size(dCSlmCoeffCols, 1) < double(ui32RequiredRows)
    error('EvalExtSphericalHarmExpCore:InsufficientCoefficients', ...
        'Coefficient table must contain at least %u rows for the requested degree.', ...
        ui32RequiredRows);
end

% Build the spherical harmonic basis and compute the potential and gradient in spherical coordinates
[dPotentialBasis, dGradUBasis, ui32CoeffRowIds, ui8CoeffColIds, dPlm] = BuildExtSphHarmExpBasis(dPosSCnorm, dSCLat, dSCLong, ...
                                                ui32MaxDegree, dGravParam, dBodyRadiusRef);

% Map coefficients matrix to linear array
dCoeffVector = zeros(numel(ui32CoeffRowIds), 1);
for idCoeff = 1:numel(ui32CoeffRowIds)
    dCoeffVector(idCoeff) = dCSlmCoeffCols(ui32CoeffRowIds(idCoeff), ui8CoeffColIds(idCoeff));
end

% Compute the potential and gradient by linear superposition of the basis contributions
dPotentialPert = dPotentialBasis * dCoeffVector;
dGradU = dGradUBasis * dCoeffVector;

end
