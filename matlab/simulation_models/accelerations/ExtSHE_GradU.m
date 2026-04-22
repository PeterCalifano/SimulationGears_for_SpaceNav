function [dGradU, dPlm] = ExtSHE_GradU(dPosSCnorm, dSCLat, dSCLong, ...
                                       ui8lMax, strParams) %#codegen
%% PROTOTYPE
% [dGradU, dPlm] = ExtSHE_GradU(dPosSCnorm, dSCLat, dSCLong, ui8lMax, strParams) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Legacy wrapper around the shared spherical-gradient core for the Exterior
% Spherical Harmonics Expansion model.
%
% This entry point preserves the original struct-based interface while the
% actual computation lives in ExtSHE_GradUCore so the MATLAB and codegen
% paths share the same implementation.
%
% ACHTUNG: Clm and Slm coefficients must be UN-NORMALIZED and are assumed
% to start from C11, S11 as first entries in the coefficient columns.
% -------------------------------------------------------------------------------------------------------------
%% INPUTS
% dPosSCnorm:   [1]      Position vector norm in the target-body fixed frame.
% dSCLat:       [1]      Geocentric latitude [rad].
% dSCLong:      [1]      Longitude [rad].
% ui8lMax:      [1]      Maximum harmonic degree.
% strParams:    [struct] Data structure containing:
%                        1) mu
%                        2) meanRbody
%                        3) Clm_LUT
%                        4) Slm_LUT
% -------------------------------------------------------------------------------------------------------------
%% OUTPUTS
% dGradU:     [3x1] Gradient of the non-spherical potential in spherical
%                   coordinates [dU/dr; dU/dLat; dU/dLong].
% dPlm:       [N x1] Flattened Legendre polynomial lookup table returned by
%                   the shared core.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ExtSHE_GradUCore()
% -------------------------------------------------------------------------------------------------------------

ui32lMax = uint32(ui8lMax);
dCSlmCoeffCols = [strParams.Clm_LUT(:), strParams.Slm_LUT(:)];

[dGradU, dPlm] = ExtSHE_GradUCore(dPosSCnorm, dSCLat, dSCLong, ...
                                  ui32lMax, dCSlmCoeffCols, strParams.mu, strParams.meanRbody);
end
