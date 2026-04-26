function dKepDot = EvalRHS_GaussPlanetary(dTime, dKepState, dGravParam, dBodyRadius, ...
    dJ2coeff, dBcoeffPoly, ui8PerturbationType)%#codegen
arguments (Input)
    dTime               (1,1) double {mustBeNumeric}
    dKepState           (6,1) double {mustBeNumeric}
    dGravParam          (1,1) double {mustBeNumeric, mustBePositive}
    dBodyRadius         (1,1) double {mustBeNumeric, mustBePositive}
    dJ2coeff            (1,1) double {mustBeNumeric}
    dBcoeffPoly         (:,1) double {mustBeNumeric}
    ui8PerturbationType (1,1) uint8  {coder.mustBeConst, mustBeMember(ui8PerturbationType, [0, 1, 2, 3])}
end
arguments (Output)
    dKepDot             (6,1) double
end
%% PROTOTYPE
% dKepDot = EvalRHS_GaussPlanetary(dTime, dKepState, dGravParam, dBodyRadius, dJ2coeff, dBcoeffPoly, ui8PerturbationType)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% ODE system for Gauss planetary equations (Keplerian elements rate of change) under perturbations.
% Perturbation types:
%   0 --> Unperturbed (Keplerian motion only)
%   1 --> J2 perturbation only
%   2 --> J2 + atmospheric drag
%   3 --> Atmospheric drag only
% State vector: [a; e; i; RAAN; omega; trueAnomaly]
%
% PERFNOTE:
% This function is on the ODE hot path. The perturbation selector is
% required to be compile-time constant for code generation so inactive
% perturbation branches can be eliminated.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTime               (1,1) double   Current time [s]
% dKepState           (6,1) double   Keplerian state [a; e; i; RAAN; omega; trueAnomaly] [km, rad]
% dGravParam          (1,1) double   Gravitational parameter [km^3/s^2]
% dBodyRadius         (1,1) double   Reference body radius [km]
% dJ2coeff            (1,1) double   J2 zonal harmonic coefficient [-]
% dBcoeffPoly         (:,1) double   Ballistic coefficient polynomial coeffs (polyval format) [m^2/kg]
% ui8PerturbationType (1,1) uint8    Perturbation type: 0=none, 1=J2, 2=J2+drag, 3=drag
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dKepDot             (6,1) double   Time derivative of Keplerian state [km/s, rad/s]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-01-2022    Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro Rizzo   Initial version.
% 28-03-2026    Pietro Califano, Claude Code    Modernized header, arguments block, naming.
% 22-04-2026    Pietro Califano, OpenAI Codex   Optimized hot path: no per-call validation, lazy perturbation evaluation,
%                                               inlined J2 computation, reduced redundant algebra.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputeAeroDrag_Gauss()
% -------------------------------------------------------------------------------------------------------------

coder.inline('always');

% Extract the state components needed on every path.
dSma = dKepState(1);
dEcc = dKepState(2);
dTrueAnom = dKepState(6);

% Common orbital quantities.
dOneMinusEcc2 = 1.0 - dEcc * dEcc;
dSemiLatRect = dSma * dOneMinusEcc2;
dSinTrueAnom = sin(dTrueAnom);
dCosTrueAnom = cos(dTrueAnom);
dSemiLatRectOverRadius = 1.0 + dEcc * dCosTrueAnom;
dRadius = dSemiLatRect / dSemiLatRectOverRadius;
dAngMomentum = sqrt(dGravParam * dSemiLatRect);
dInvAngMomentum = 1.0 / dAngMomentum;
dRadius2 = dRadius * dRadius;

dKepDot = zeros(6, 1);
dKepDot(6) = dAngMomentum / dRadius2;

% Fast exit for the unperturbed case.
if ui8PerturbationType == 0
    return;
end

% Extract the remaining orbital elements only when a perturbation term is active.
dIncl = dKepState(3);
dRaan = dKepState(4);
dArgPeri = dKepState(5);
dArgLat = dTrueAnom + dArgPeri;

dAccR = 0.0;
dAccS = 0.0;
dAccW = 0.0;

% J2 contribution: inline the simple algebra instead of paying another function call.
if coder.const(ui8PerturbationType ~= 3) && dJ2coeff ~= 0.0

    dSinIncl = sin(dIncl);
    dCosIncl = cos(dIncl);
    dSinArgLat = sin(dArgLat);
    dCosArgLat = cos(dArgLat);
    dSinIncl2 = dSinIncl * dSinIncl;
    dSinArgLat2 = dSinArgLat * dSinArgLat;
    dInvRadius4 = 1.0 / (dRadius2 * dRadius2);
    dCommonFactor = -1.5 * dJ2coeff * dGravParam * (dBodyRadius * dBodyRadius) * dInvRadius4;

    dAccR = dCommonFactor * (1.0 - 3.0 * dSinIncl2 * dSinArgLat2);
    dAccS = dCommonFactor * dSinIncl2 * (2.0 * dSinArgLat * dCosArgLat);
    dAccW = dCommonFactor * (2.0 * dSinIncl * dCosIncl) * dSinArgLat;
end

% Drag contribution: only evaluate the ballistic coefficient and the expensive
% Keplerian->Cartesian transform when drag is enabled.
if coder.const(ui8PerturbationType >= 2)
    if numel(dBcoeffPoly) == 1
        dBvalue = dBcoeffPoly(1);
    else
        dBvalue = polyval(dBcoeffPoly, dTime);
    end

    if dBvalue ~= 0.0
        dAccDrag_RSW = ComputeAeroDrag_Gauss( ...
            dSma, dEcc, dIncl, dRaan, dArgPeri, dTrueAnom, ...
            dGravParam, dBvalue, dBodyRadius);

        dAccR = dAccR + dAccDrag_RSW(1);
        dAccS = dAccS + dAccDrag_RSW(2);
        dAccW = dAccW + dAccDrag_RSW(3);
    end
end

% Remaining Gauss variational equations.
dSemiLatRectPlusRadius = dSemiLatRect + dRadius;
dInvEccAngMomentum = 1.0 / (dEcc * dAngMomentum);

% Compute SMA and eccentricity rates (always needed)
dKepDot(1) = (2.0 * dSma * dSma * dInvAngMomentum) * ...
    (dEcc * dSinTrueAnom * dAccR + dSemiLatRectOverRadius * dAccS);
dKepDot(2) = dInvAngMomentum * ...
    (dSemiLatRect * dSinTrueAnom * dAccR + ...
    (dSemiLatRectPlusRadius * dCosTrueAnom + dRadius * dEcc) * dAccS);

% Compute inclination, RAAN, and argument of pericentre rates only if out-of-plane acceleration is significant, otherwise skip to save flops.
if abs(dAccW) >= coder.const(1E-3 * eps('double'))
    % Full equations when out-of-plane acceleration is significant
    dSinIncl = sin(dIncl);
    dCosIncl = cos(dIncl);
    dSinArgLat = sin(dArgLat);
    dCosArgLat = cos(dArgLat);
    dRadiusOverAngMomentum = dRadius * dInvAngMomentum;
    dInvAngMomentumSinIncl = dInvAngMomentum / dSinIncl;

    dKepDot(3) = dRadiusOverAngMomentum * dCosArgLat * dAccW;
    dKepDot(4) = dRadius * dSinArgLat * dAccW * dInvAngMomentumSinIncl;
    dKepDot(5) = dInvEccAngMomentum * ...
        (-dSemiLatRect * dCosTrueAnom * dAccR + dSemiLatRectPlusRadius * dSinTrueAnom * dAccS) - ...
        (dRadius * dSinArgLat * dCosIncl * dAccW) * dInvAngMomentumSinIncl;
else
    % Simplified equations when out-of-plane acceleration is negligible
    dKepDot(5) = dInvEccAngMomentum * ...
        (-dSemiLatRect * dCosTrueAnom * dAccR + dSemiLatRectPlusRadius * dSinTrueAnom * dAccS);
end

% Compute true anomaly rate
dKepDot(6) = dKepDot(6) + dInvEccAngMomentum * ...
    (dSemiLatRect * dCosTrueAnom * dAccR - dSemiLatRectPlusRadius * dSinTrueAnom * dAccS);

end
