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
% 05-10-2026  Pietro Califano, Codex (GPT-6)        Reuse the complete response derivatives.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveAttQuat_INfromSCB, ResolvePanelSRPUnitsFromDynParams,
% ComputePanelSrpResponse, Quat2DCM [MathCore_for_SpaceNav].
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
dDCM_INfromSCB = Quat2DCM(dQuat_INfromSCB);
dSunVector_SCB = dDCM_INfromSCB.' * dSCtoSun_IN;

[dArea, dPressCentre, ~, dPressureSI, dOutputScale] = ...
    ResolvePanelSRPUnitsFromDynParams(strPanel, zeros(3, 1), dSolarPressure, strDynParams);

strPanel.dSCquadsArea = dArea;
strPanel.dQuadsPressCentre_SCB = dPressCentre;

% Differentiate the supplied Sun vector while holding sampled visibility fixed.
[dForce, dResponseJacobian] = ComputePanelSrpResponse(dSunVector_SCB, strPanel, true);
dScale = dOutputScale * dPressureSI / strDynParams.strSCdata.dSCmass;
dJacPanelSRP_IN = -dScale * dDCM_INfromSCB * dResponseJacobian * dDCM_INfromSCB.';

if bRecomputePressureFromDistance
    dAcceleration = dScale * (dDCM_INfromSCB * dForce);
    dJacPanelSRP_IN = dJacPanelSRP_IN + 2 * dAcceleration * dSCtoSun_IN.' / dot(dSCtoSun_IN, dSCtoSun_IN);
end

end
