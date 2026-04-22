function dSHEaccTB = ExtSHE_AccTB(dPosSC_TB, ...
                                ui32MaxDegree, ...
                                dCSlmCoeffCols, ...
                                dGravParam, ...
                                dBodyRadiusRef) %#codegen
arguments
    dPosSC_TB           (3,1) double
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,:) double
    dGravParam          (1,1) double
    dBodyRadiusRef      (1,1) double
end
%% PROTOTYPE
% dSHEaccTB = ExtSHE_AccTB(dPosSC_TB, ui32MaxDegree, dCSlmCoeffCols, ...
%                          dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function implementing the Exterior Spherical Harmonics Expansion model to
% compute the perturbative gravitational acceleration in the target-body
% fixed frame.
%
% The implementation evaluates the spherical-coordinate gradient through
% ExtSHE_GradUCore and maps it to Cartesian coordinates directly. This
% preserves the original ExtSHE interface while avoiding the singular
% chain-rule matrix used by the earlier implementation.
%
% ACHTUNG: Clm and Slm coefficients must be UN-NORMALIZED and stored as
% [Clm, Slm] column pairs starting from (l,m) = (1,1). Degree-1 entries are
% accepted for format compatibility but are ignored by the model, which
% starts at degree 2.
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUTS
% dPosSC_TB:        [3x1]   Spacecraft position vector in the target-body
%                           fixed frame.
% ui32MaxDegree:    [1]     Maximum harmonic degree to be considered.
% dCSlmCoeffCols:   [Nl, 2] Unnormalized [Clm, Slm] coefficient lookup table.
% dGravParam:       [1]     Body gravitational parameter [LU^3/TU^2]
% dBodyRadiusRef:   [1]     Reference body radius [LU]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUTS
% dSHEaccTB:        [3x1]   Non-spherical acceleration vector in the
%                           target-body fixed frame.
% -------------------------------------------------------------------------------------------------------------
%% NOTES
% 1) This function returns only the non-spherical contribution. The
%    central-force term must be added separately if required.
% 2) Pole handling is treated explicitly when the in-plane radius tends to
%    zero.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ExtSHE_GradUCore()
% -------------------------------------------------------------------------------------------------------------

% Compute latitude, longitude and distance
dPosSCnorm = sqrt(dPosSC_TB(1)^2 + dPosSC_TB(2)^2 + dPosSC_TB(3)^2);
dSCLat = asin(dPosSC_TB(3) / dPosSCnorm);
dSCLong = atan2(dPosSC_TB(2), dPosSC_TB(1));

% Evaluate gradient of SH model
dGradU = ExtSHE_GradUCore(dPosSCnorm, dSCLat, dSCLong, ui32MaxDegree, ...
                            dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

dUdr = dGradU(1);
dUdLat = dGradU(2);
dUdLong = dGradU(3);

dRho2 = dPosSC_TB(1)^2 + dPosSC_TB(2)^2;

% Check numerical thresholds
if dRho2 <= (16.0 * eps(dPosSCnorm) * dPosSCnorm)^2
    dSHEaccTB = [0.0; 0.0; dUdr * dPosSC_TB(3) / dPosSCnorm];
    return;
end

% Precompute auxiliaries and shared quantities
dRho = sqrt(dRho2);
dInvR = 1.0 / dPosSCnorm;
dInvR2 = dInvR * dInvR;

dCommon = dUdr * dInvR - dPosSC_TB(3) * dUdLat / (dPosSCnorm^2 * dRho);

% Evaluate acceleration in target fixed frame dAccTB = dGradU/drSC_TB
dSHEaccTB = zeros(3, 1);
dSHEaccTB(1) = dCommon * dPosSC_TB(1) - dUdLong * dPosSC_TB(2) / dRho2;
dSHEaccTB(2) = dCommon * dPosSC_TB(2) + dUdLong * dPosSC_TB(1) / dRho2;
dSHEaccTB(3) = dUdr * dPosSC_TB(3) * dInvR + dUdLat * dRho * dInvR2;

end
