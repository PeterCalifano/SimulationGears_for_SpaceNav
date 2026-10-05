function dJacPanelSRP_IN = ComputePanelSRPJacobianFromDynParams( ...
    dPosSC_IN, dSunPos_IN, dSolarPressure, strDynParams, ...
    bRecomputePressureFromDistance) %#codegen
%% SIGNATURE
% dJacPanelSRP_IN = ComputePanelSRPJacobianFromDynParams( ...
%     dPosSC_IN, dSunPos_IN, dSolarPressure, strDynParams, bRecomputePressureFromDistance)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute panel acceleration position partials in max-fidelity dynamics units.
% Apply the same sampled visibility fractions as the RHS, then hold them fixed
% while differentiating the panel law. Hold attitude, Sun position, geometry,
% mass and optics fixed. Hard quadrature visibility changes have no derivative
% at switching boundaries; return a local branch linearization.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_IN                      (3,1) Spacecraft target-relative position [LU].
% dSunPos_IN                     (3,1) Sun target-relative position [LU].
% dSolarPressure                 (1,1) Current dynamics pressure.
% strDynParams                  Panel payload with optional numeric strShadowData.
% bRecomputePressureFromDistance Include live inverse-square pressure partials.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacPanelSRP_IN                (3,3) Inertial d(a_panel)/d(r_SC) [s^-2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026  Pietro Califano, Codex 5.5    Extract panel SRP Jacobian adapter.
% 04-10-2026  Pietro Califano, Codex GPT-6  Freeze prepared self-shadow visibility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveAttQuat_INfromSCB, ResolvePanelSRPUnitsFromDynParams,
% ComputePanelVisibleAreas, EvalJac_QuadsModelSRP, Quat2DCM [MathCore_for_SpaceNav].
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dPosSC_IN (3, 1) double {mustBeFinite}
    dSunPos_IN (3, 1) double {mustBeFinite}
    dSolarPressure (1, 1) double {mustBeFinite, mustBeNonnegative}
    strDynParams (1, 1) struct
    bRecomputePressureFromDistance (1, 1) logical
end

arguments (Output)
    dJacPanelSRP_IN (3, 3) double
end

% Use the force adapter's normalized frame and visibility at this position.
strPanel = strDynParams.strSCdata.strSRPpanelData;
dQuat_INfromSCB = ResolveAttQuat_INfromSCB(strDynParams);
dQuat_INfromSCB = dQuat_INfromSCB / max(norm(dQuat_INfromSCB), eps);
dSCtoSun_IN = dSunPos_IN - dPosSC_IN;
dSunDir_SCB = Quat2DCM(dQuat_INfromSCB).' * (dSCtoSun_IN / norm(dSCtoSun_IN));

[dArea, ~, ~, dPressureSI, dOutputScale] = ...
    ResolvePanelSRPUnitsFromDynParams(strPanel, zeros(3, 1), dSolarPressure, strDynParams);
dVisibleArea = ComputePanelVisibleAreas(dArea, dSunDir_SCB, strPanel);

% Differentiate incidence/direction and optional live pressure on the current
% visibility branch, leaving ray-hit and terminator switches undifferentiated.
dJacPanel = EvalJac_QuadsModelSRP(dSCtoSun_IN, dQuat_INfromSCB, ...
    strDynParams.strSCdata.dSCmass, dPressureSI, dVisibleArea, ...
    strPanel.dDiffSpecQuadsCoeffs, strPanel.dQuadsNormals_SCB, ...
    false, bRecomputePressureFromDistance);
dJacPanelSRP_IN = dOutputScale * dJacPanel;

end
