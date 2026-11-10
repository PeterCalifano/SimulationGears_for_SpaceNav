function [dP_SRP, dP_SRP0] = ComputeSolarRadPressure(dInvNormSunPositionFromSC, ...
                                                  bUseKilometersScale, ...
                                                  dReferencePressure) %#codegen
%% SIGNATURE
% [dP_SRP, dP_SRP0] = ComputeSolarRadPressure(dInvNormSunPositionFromSC, ...
%                                           bUseKilometersScale, dReferencePressure)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Scale a reference solar pressure at 1 AU by the inverse square of Sun-spacecraft distance.
% Supply the reference pressure in units consistent with the selected length scale. Retain the
% nominal 1367 W/m^2 irradiance divided by light speed when the third argument is omitted.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dInvNormSunPositionFromSC (1,1) double   Positive inverse distance [1/m or 1/km].
% bUseKilometersScale       (1,1) logical Constant length-scale selection; default false.
% dReferencePressure        (1,1) double   Nonnegative pressure at 1 AU [kg/(m*s^2) or
%                                       kg/(km*s^2)]. Multiply SI pressure by 1e3 for kilometre
%                                       dynamics; this is not a pressure in N/km^2.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dP_SRP                    (1,1) double   Pressure at spacecraft distance in selected units.
% dP_SRP0                   (1,1) double   Unchanged reference pressure at 1 AU in selected units.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-12-2025    Pietro Califano     First implementation from previous code. Part of refactoring of
%                                   EstimationGears repository for better usage and validation.
% 28-09-2026    Codex               Accept configured reference pressure; retain nominal legacy calls.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dInvNormSunPositionFromSC (1,1) double {mustBeFinite, mustBePositive}
    bUseKilometersScale       (1,1) logical {coder.mustBeConst} = false
    dReferencePressure        (1,1) double {mustBeFinite, mustBeNonnegative} = ...
        (1367 / 299792458) * 1e3^double(bUseKilometersScale)
end

arguments (Output)
    dP_SRP  (1,1) double
    dP_SRP0 (1,1) double
end

%% Function code

% Match the astronomical-unit distance to the caller's dynamics length scale.
if coder.const(bUseKilometersScale)
    dAU = coder.const(1.495978707E8);
else
    dAU = coder.const(1.495978707E11);
end

% Preserve the supplied reference while applying only geometric attenuation.
dAU2 = coder.const(dAU * dAU);
dP_SRP0 = dReferencePressure;
dP_SRP = dP_SRP0 * (dAU2 * dInvNormSunPositionFromSC^2);

end
