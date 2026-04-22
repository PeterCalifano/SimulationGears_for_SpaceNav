function dAccJ2_RSW = ComputeJ2Perturbation_Gauss(dSma, ...
                                                dEcc, ...
                                                dIncl, ...
                                                dArgPeri, ...
                                                dTrueAnom, ...
                                                dBodyRadius, ...
                                                dGravParam, ...
                                                dJ2coeff)%#codegen
arguments (Input)
    dSma        (1,1) double {mustBeNumeric, mustBePositive}
    dEcc        (1,1) double {mustBeNumeric, mustBeNonnegative}
    dIncl       (1,1) double {mustBeNumeric}
    dArgPeri    (1,1) double {mustBeNumeric}
    dTrueAnom   (1,1) double {mustBeNumeric}
    dBodyRadius (1,1) double {mustBeNumeric, mustBePositive}
    dGravParam  (1,1) double {mustBeNumeric, mustBePositive}
    dJ2coeff    (1,1) double {mustBeNumeric}
end
arguments (Output)
    dAccJ2_RSW  (3,1) double
end
%% PROTOTYPE
% dAccJ2_RSW = ComputeJ2Perturbation_Gauss(dSma, dEcc, dIncl, dArgPeri, dTrueAnom, dBodyRadius, dGravParam, dJ2coeff)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes the J2 zonal harmonic perturbation acceleration in the RSW (Radial, Along-track, Cross-track)
% frame for use with Gauss planetary equations. Angles are expected in radians.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSma        (1,1) double   Semi-major axis [km]
% dEcc        (1,1) double   Eccentricity [-]
% dIncl       (1,1) double   Inclination [rad]
% dArgPeri    (1,1) double   Argument of pericentre [rad]
% dTrueAnom   (1,1) double   True anomaly [rad]
% dBodyRadius (1,1) double   Central body equatorial radius [km]
% dGravParam  (1,1) double   Central body gravitational parameter [km^3/s^2]
% dJ2coeff    (1,1) double   J2 zonal harmonic coefficient [-]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccJ2_RSW  (3,1) double   J2 acceleration in RSW frame [km/s^2]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-01-2022    Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro Rizzo   Initial version.
% 28-03-2026    Claude Code       Modernized header, arguments block, naming.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

% Compute orbital radius from Keplerian elements
dSemiLatRect = dSma * (1 - dEcc^2);
dRadius      = dSemiLatRect / (1 + dEcc * cos(dTrueAnom));

% Argument of latitude
dArgLat = dTrueAnom + dArgPeri;

% J2 common factor: -3/2 * J2 * mu * R^2 / r^4
dCommonFactor = -1.5 * dJ2coeff * dGravParam * dBodyRadius^2 / dRadius^4;

% RSW acceleration components
dSinArgLat = sin(dArgLat);
dSinIncl = sin(dIncl);

dAccR = dCommonFactor * (1 - 3 * dSinIncl^2 * dSinArgLat^2);
dAccS = dCommonFactor * dSinIncl^2 * sin(2 * dArgLat);
dAccW = dCommonFactor * sin(2 * dIncl) * dSinArgLat;

dAccJ2_RSW = [dAccR; dAccS; dAccW];

end
