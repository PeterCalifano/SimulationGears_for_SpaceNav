function [dAccPanelSRP_IN, dSRPtorque_SCB] = ComputePanelSRPFromDynParams(dPosSC_IN, ...
                                                                          dSunPos_IN, ...
                                                                          dSolarPressure, ...
                                                                          strDynParams) %#codegen
arguments
    dPosSC_IN      (3,1) double {mustBeFinite}
    dSunPos_IN     (3,1) double {mustBeFinite}
    dSolarPressure (1,1) double {mustBeFinite, mustBeNonnegative}
    strDynParams   (1,1) struct
end
%% PROTOTYPE
% [dAccPanelSRP_IN, dSRPtorque_SCB] = ComputePanelSRPFromDynParams(dPosSC_IN, dSunPos_IN, dSolarPressure, strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute flat-panel SRP acceleration and torque from the max-fidelity dynamics payload.
% This adapter resolves spacecraft attitude, center of mass, and unit normalization before calling
% ComputeQuadsModelSRP.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_IN:      (3,1) double   Spacecraft position with respect to the target in inertial frame [LU].
% dSunPos_IN:     (3,1) double   Sun position with respect to the target in inertial frame [LU].
% dSolarPressure: (1,1) double   Current solar pressure from the dynamics payload.
% strDynParams:   (1,1) struct   Dynamics payload with strSCdata.strSRPpanelData.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccPanelSRP_IN:(3,1) double   Panel SRP acceleration in inertial dynamics units [LU/TU^2].
% dSRPtorque_SCB: (3,1) double   Panel SRP torque in spacecraft body frame [N m].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract max-fidelity flat-panel SRP RHS adapter.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveAttQuat_INfromSCB()
% ResolveSCCenterOfMass_SCB()
% ResolvePanelSRPUnitsFromDynParams()
% ComputeQuadsModelSRP()
% Quat2DCM()   [MathCore_for_SpaceNav]
% -------------------------------------------------------------------------------------------------------------

%% Function code
strPanel = strDynParams.strSCdata.strSRPpanelData;
dqSCBwrtIN = ResolveAttQuat_INfromSCB(strDynParams);
dCoMpos_SCB = ResolveSCCenterOfMass_SCB(strDynParams);

dSCtoSun_IN = dSunPos_IN - dPosSC_IN;
dDirSCtoSun_IN = dSCtoSun_IN / norm(dSCtoSun_IN);
dDCM_INfromSCB = Quat2DCM(dqSCBwrtIN);
dDirSCtoSun_SCB = dDCM_INfromSCB.' * dDirSCtoSun_IN;

[dArea, dPressCentre, dCoMpos, dPressureSI, dOutputScale] = ...
    ResolvePanelSRPUnitsFromDynParams(strPanel, ...
                                      dCoMpos_SCB, ...
                                      dSolarPressure, ...
                                      strDynParams);

assert(size(dPressCentre, 2) == size(strPanel.dQuadsNormals_SCB, 2), ...
    'ComputePanelSRPFromDynParams:MissingPressureCenters', ...
    'Panel SRP RHS requires strSRPpanelData.dQuadsPressCentre_SCB with one pressure-center column per panel.');

[dAccelPanel, dSRPtorque_SCB] = ComputeQuadsModelSRP(dDirSCtoSun_SCB, ...
                                                     dqSCBwrtIN, ...
                                                     strDynParams.strSCdata.dSCmass, ...
                                                     dCoMpos, ...
                                                     dPressureSI, ...
                                                     dArea, ...
                                                     strPanel.dDiffSpecQuadsCoeffs, ...
                                                     strPanel.dQuadsNormals_SCB, ...
                                                     dPressCentre);
dAccPanelSRP_IN = dOutputScale * dAccelPanel;

end
