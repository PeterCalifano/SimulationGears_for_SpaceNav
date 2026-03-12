function [dP_SRP, dP_SRP0] = ComputeSolarRadPressure(dInvNormSunPositionFromSC, ...
                                                    bUseKilometersScale)%#codegen
arguments
    dInvNormSunPositionFromSC (1,1) double  {mustBePositive}
    bUseKilometersScale       (1,1) logical {coder.mustBeConst} = false
end
%% PROTOTYPE
% [dP_SRP, dP_SRP0] = ComputeSolarRadPressure(dInvNormSunPositionFromSC, ...
%                                                     bUseKilometersScale)%#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function computing the average Solar radiation pressure value scaling depending on distance of
% spacecraft from the Sun. Average P_SRP assuming 1367 W/m^2 at Earth @ 1 AU. Scaling to km is performed
% depending on configuration (coder constant).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dInvNormSunPositionFromSC (1,1) double  {mustBePositive}
% bUseKilometersScale       (1,1) logical {coder.mustBeConst} = false
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% drvSRPwithBiasJac
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-12-2025    Pietro Califano     First implementation from previous code. Part of refactoring of
%                                   EstimationGears repository for better usage and validation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
if coder.const(bUseKilometersScale)
    % Assumes km scale
    dAU = coder.const(1.495978707E8);
    dP_SRP0 = coder.const(1E3 * 1367 / 299792458);
else
    % Assumes m scale
    dAU = coder.const(1.495978707E11);
    dP_SRP0 = coder.const(1367 / 299792458); % Approx. 4.54e-6 N/m^2
end

% Compute AU^2 as coder constant
dAU2 = coder.const(dAU * dAU);

% Compute SRP value from SRP0 at 1AU
dP_SRP = dP_SRP0 * (dAU2 * (dInvNormSunPositionFromSC^2)); % [N/m^2] or [N/km^2]

end

