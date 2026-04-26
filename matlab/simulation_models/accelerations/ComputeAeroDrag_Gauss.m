function dAccDrag_RSW = ComputeAeroDrag_Gauss(dSma, ...
                                                dEcc, ...
                                                dIncl, ...
                                                dRaan, ...
                                                dArgPeri, ...
                                                dTrueAnom, ...
                                                dGravParam, ...
                                                dBcoeff, ...
                                                dBodyRadius)%#codegen
arguments (Input)
    dSma        (1,1) double {mustBeNumeric, mustBePositive}
    dEcc        (1,1) double {mustBeNumeric, mustBeNonnegative}
    dIncl       (1,1) double {mustBeNumeric}
    dRaan       (1,1) double {mustBeNumeric}
    dArgPeri    (1,1) double {mustBeNumeric}
    dTrueAnom   (1,1) double {mustBeNumeric}
    dGravParam  (1,1) double {mustBeNumeric, mustBePositive}
    dBcoeff     (1,1) double {mustBeNumeric}
    dBodyRadius (1,1) double {mustBeNumeric, mustBePositive}
end
arguments (Output)
    dAccDrag_RSW (3,1) double
end
%% PROTOTYPE
% dAccDrag_RSW = ComputeAeroDrag_Gauss(dSma, dEcc, dIncl, dRaan, dArgPeri, dTrueAnom, dGravParam, dBcoeff, dBodyRadius)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes atmospheric drag acceleration in the RSW (Radial, Along-track, Cross-track) frame for use with
% Gauss planetary equations. Uses an exponential atmosphere model with tabulated reference densities and
% scale heights, extended by exponential extrapolation up to 1300 km and forced to zero above that limit.
% Angles are in radians. Earth rotation is included for relative velocity computation.
% DEVNOTE: Atmosphere model data from Vallado, "Fundamentals of Astrodynamics and Applications".
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSma        (1,1) double   Semi-major axis [km]
% dEcc        (1,1) double   Eccentricity [-]
% dIncl       (1,1) double   Inclination [rad]
% dRaan       (1,1) double   RAAN [rad]
% dArgPeri    (1,1) double   Argument of pericentre [rad]
% dTrueAnom   (1,1) double   True anomaly [rad]
% dGravParam  (1,1) double   Gravitational parameter [km^3/s^2]
% dBcoeff     (1,1) double   Ballistic coefficient A*Cd/m [m^2/kg]
% dBodyRadius (1,1) double   Central body equatorial radius [km]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccDrag_RSW (3,1) double   Drag acceleration in RSW frame [km/s^2]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-01-2022    Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro Rizzo   Initial version.
% 28-03-2026    Claude Code     Modernized: lookup table, header, arguments, naming.
% 22-04-2026    OpenAI Codex    Extended exponential table to 1300 km and cached reusable constants.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% kepl2rv()   [MathCore_for_SpaceNav]
% -------------------------------------------------------------------------------------------------------------

% Initialize to zero
dAccDrag_RSW = zeros(3, 1);
if abs(dBcoeff) <= coder.const(0.001 * eps('double'))
    return;
end

[dAtmosphereTable, dRefAltitudes, dEarthAngVel, dMinAltitude, dMaxAltitude] = getAtmosphereModel_();

% Compute orbital radius directly from the Keplerian elements so out-of-range
% altitudes can return early without paying for a Cartesian conversion.
dSemiLatRect = dSma * (1 - dEcc^2);
dRadius      = dSemiLatRect / (1 + dEcc * cos(dTrueAnom));
dAltitude    = dRadius - dBodyRadius; % [km]

if dAltitude < dMinAltitude || dAltitude > dMaxAltitude
    % Atmosphere model is only defined between 70 km and 1300 km.
    return;
end

% Find applicable atmosphere layer (largest dRefAltitude <= dAltitude)
idxLayer = find(dRefAltitudes <= dAltitude, 1, 'last');

dRefAlt      = dAtmosphereTable(idxLayer, 1);
dRefDensity  = dAtmosphereTable(idxLayer, 2);
dScaleHeight = dAtmosphereTable(idxLayer, 3);

% Exponential density model: rho = rho0 * exp(-(h - h0) / H)
dDensity = dRefDensity * exp(-(dAltitude - dRefAlt) / dScaleHeight); % [kg/m^3]

% Convert Keplerian elements to Cartesian (MathCore kepl2rv convention: [SMA;Ecc;Incl;RAAN;ArgPeri;TA])
dxCart = kepl2rv([dSma; dEcc; dIncl; dRaan; dArgPeri; dTrueAnom], dGravParam);
dPosVec_ECEI = dxCart(1:3);
dVelVec_ECEI = dxCart(4:6);

% Relative velocity in ECEI (accounting for Earth rotation), converted to [m/s]
dRelVel_ECEI = (dVelVec_ECEI - cross(dEarthAngVel, dPosVec_ECEI)) * 1e3;
dRelSpeed    = norm(dRelVel_ECEI);

% Drag acceleration in ECEI [m/s^2]
dAccDrag_ECEI = -0.5 * dBcoeff * dDensity * dRelSpeed * dRelVel_ECEI;

% Rotation from ECEI to RSW frame
dRadialUnit     = dPosVec_ECEI / dRadius;
dAngMomVec      = cross(dPosVec_ECEI, dVelVec_ECEI);
dCrossTrackUnit = dAngMomVec / norm(dAngMomVec);
dAlongTrackUnit = cross(dCrossTrackUnit, dRadialUnit);

dDCM_RSWfromECEI = [dRadialUnit, dAlongTrackUnit, dCrossTrackUnit]';

% Rotate to RSW and convert to [km/s^2]
dAccDrag_RSW = dDCM_RSWfromECEI * dAccDrag_ECEI * 1e-3;

end

%% Internal helpers
function [dAtmosphereTable, dRefAltitudes, dEarthAngVel, dMinAltitude, dMaxAltitude] = getAtmosphereModel_()

persistent dCachedAtmosphereTable dCachedRefAltitudes dCachedEarthAngVel dCachedMinAltitude dCachedMaxAltitude

if isempty(dCachedAtmosphereTable)
    % Exponential atmosphere model lookup table (Vallado)
    % Columns: [dRefAltitude_km, dRefDensity_kg_m3, dScaleHeight_km]
    dBaseAtmosphereTable = [70, 8.770e-05,   6.549;
                            80, 1.905e-05,   5.799;
                            90, 3.396e-06,   5.382;
                            100, 5.297e-07,   5.877;
                            110, 9.661e-08,   7.263;
                            120, 2.438e-08,   9.473;
                            130, 8.484e-09,  12.636;
                            140, 3.845e-09,  16.149;
                            150, 2.070e-09,  22.523;
                            180, 5.464e-10,  29.740;
                            200, 2.789e-10,  37.105;
                            250, 7.248e-11,  45.546;
                            300, 2.418e-11,  53.628;
                            350, 9.158e-12,  53.298;
                            400, 3.725e-12,  58.515;
                            450, 1.585e-12,  60.828;
                            500, 6.967e-13,  63.922;
                            600, 1.454e-13,  71.835;
                            700, 3.614e-14,  88.667;
                            800, 1.170e-14, 124.640;
                            900, 5.245e-15, 181.050;
                            1000, 3.019e-15, 268.000];

    dExtrapAltitudes = (1100:100:1300).';
    dLastRefAlt      = dBaseAtmosphereTable(end, 1);
    dLastRefDensity  = dBaseAtmosphereTable(end, 2);
    dLastScaleHeight = dBaseAtmosphereTable(end, 3);

    dExtrapDensities    = dLastRefDensity * exp(-(dExtrapAltitudes - dLastRefAlt) / dLastScaleHeight);
    dExtrapScaleHeights = dLastScaleHeight * ones(numel(dExtrapAltitudes), 1);

    dCachedAtmosphereTable = [dBaseAtmosphereTable; ...
                              [dExtrapAltitudes, dExtrapDensities, dExtrapScaleHeights]];
    dCachedRefAltitudes = dCachedAtmosphereTable(:, 1);
    dCachedEarthAngVel  = [0; 0; 7.292e-5];
    dCachedMinAltitude  = dCachedRefAltitudes(1);
    dCachedMaxAltitude  = dCachedRefAltitudes(end);
end

dAtmosphereTable = dCachedAtmosphereTable;
dRefAltitudes    = dCachedRefAltitudes;
dEarthAngVel     = dCachedEarthAngVel;
dMinAltitude     = dCachedMinAltitude;
dMaxAltitude     = dCachedMaxAltitude;

end
