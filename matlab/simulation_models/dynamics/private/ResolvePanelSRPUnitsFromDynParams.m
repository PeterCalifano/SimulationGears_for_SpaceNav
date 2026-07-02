function [dArea, dPressCentre, dCoMpos, dPressureSI, dOutputScale] = ResolvePanelSRPUnitsFromDynParams(strPanel, ...
                                                                                                        dCoMpos_SCB, ...
                                                                                                        dSolarPressure, ...
                                                                                                        strDynParams) %#codegen
arguments
    strPanel       (1,1) struct
    dCoMpos_SCB    (3,1) double {mustBeFinite}
    dSolarPressure (1,1) double {mustBeFinite, mustBeNonnegative}
    strDynParams   (1,1) struct
end
%% PROTOTYPE
% [dArea, dPressCentre, dCoMpos, dPressureSI, dOutputScale] = ResolvePanelSRPUnitsFromDynParams(strPanel, ...
%     dCoMpos_SCB, dSolarPressure, strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Normalize flat-panel SRP geometry, pressure, and acceleration units for max-fidelity dynamics.
% Panel geometry is converted to SI before calling the panel SRP law. If the dynamics payload uses kilometer
% distances, the pressure and returned acceleration are scaled so RHS and Jacobian use the same unit convention.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strPanel:       (1,1) struct   Panel SRP geometry and optical data.
% dCoMpos_SCB:    (3,1) double   Spacecraft center-of-mass position in panel length units.
% dSolarPressure: (1,1) double   Current solar pressure from the dynamics payload.
% strDynParams:   (1,1) struct   Dynamics payload with SRP reference-distance unit convention.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dArea:          (N,1) double   Surface-element areas in SI units [m^2].
% dPressCentre:   (3,N) double   Surface pressure-center positions in SI units [m].
% dCoMpos:        (3,1) double   Spacecraft center-of-mass position in SI units [m].
% dPressureSI:    (1,1) double   Solar pressure normalized for the panel SRP law.
% dOutputScale:   (1,1) double   Scale from panel-law acceleration units back to dynamics acceleration units.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract shared flat-panel SRP unit normalization.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
charPanelUnit = "m";

if coder.const(isfield(strPanel, 'charLengthUnit'))
    charPanelUnit = string(strPanel.charLengthUnit);
end

dArea = strPanel.dSCquadsArea;
dPressCentre = zeros(3, 0);
dCoMpos = dCoMpos_SCB;

if coder.const(isfield(strPanel, 'dQuadsPressCentre_SCB'))
    dPressCentre = strPanel.dQuadsPressCentre_SCB;
end

if charPanelUnit == "km"
    dArea = dArea * 1e6;
    dPressCentre = dPressCentre * 1e3;
    dCoMpos = dCoMpos * 1e3;
end

bDynamicsInKm = coder.const(isfield(strDynParams.strSRPdata, 'dReferenceDistance')) && ...
    strDynParams.strSRPdata.dReferenceDistance < 1.0e10;

if bDynamicsInKm
    dPressureSI = dSolarPressure / 1e3;
    dOutputScale = 1e-3;
else
    dPressureSI = dSolarPressure;
    dOutputScale = 1.0;
end

end
