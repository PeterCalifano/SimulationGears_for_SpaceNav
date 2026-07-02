function dJacPanelSRP_IN = ComputePanelSRPJacobianFromDynParams(dPosSC_IN, ...
                                                                dSunPos_IN, ...
                                                                dSolarPressure, ...
                                                                strDynParams, ...
                                                                bRecomputePressureFromDistance) %#codegen
arguments
    dPosSC_IN                       (3,1) double {mustBeFinite}
    dSunPos_IN                      (3,1) double {mustBeFinite}
    dSolarPressure                  (1,1) double {mustBeFinite, mustBeNonnegative}
    strDynParams                    (1,1) struct
    bRecomputePressureFromDistance  (1,1) logical
end
%% PROTOTYPE
% dJacPanelSRP_IN = ComputePanelSRPJacobianFromDynParams(dPosSC_IN, dSunPos_IN, dSolarPressure, strDynParams, ...
%     bRecomputePressureFromDistance)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute the flat-panel SRP acceleration position partial from the max-fidelity dynamics payload.
% The unit normalization is shared with the RHS adapter so the Jacobian is expressed in the same dynamics units
% as ComputePanelSRPFromDynParams.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_IN:                      (3,1) double   Spacecraft target-relative inertial position [LU].
% dSunPos_IN:                     (3,1) double   Sun target-relative inertial position [LU].
% dSolarPressure:                 (1,1) double   Current solar pressure from the dynamics payload.
% strDynParams:                   (1,1) struct   Dynamics payload with strSCdata.strSRPpanelData.
% bRecomputePressureFromDistance: (1,1) logical  Include inverse-square pressure partial if true.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacPanelSRP_IN:                (3,3) double   d(a_panel_SRP)/d(r_SC) in inertial dynamics units [1/TU^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Extract max-fidelity flat-panel SRP Jacobian adapter.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveAttQuat_INfromSCB()
% ResolvePanelSRPUnitsFromDynParams()
% EvalJac_QuadsModelSRP()
% -------------------------------------------------------------------------------------------------------------

%% Function code
strPanel = strDynParams.strSCdata.strSRPpanelData;
dqSCBwrtIN = ResolveAttQuat_INfromSCB(strDynParams);
dSCtoSun_IN = dSunPos_IN - dPosSC_IN;

[dArea, ~, ~, dPressureSI, dOutputScale] = ...
    ResolvePanelSRPUnitsFromDynParams(strPanel, ...
                                      zeros(3, 1), ...
                                      dSolarPressure, ...
                                      strDynParams);

dJacPanel = EvalJac_QuadsModelSRP(dSCtoSun_IN, ...
                                  dqSCBwrtIN, ...
                                  strDynParams.strSCdata.dSCmass, ...
                                  dPressureSI, ...
                                  dArea, ...
                                  strPanel.dDiffSpecQuadsCoeffs, ...
                                  strPanel.dQuadsNormals_SCB, ...
                                  false, ...
                                  bRecomputePressureFromDistance);

dJacPanelSRP_IN = dOutputScale * dJacPanel;

end
