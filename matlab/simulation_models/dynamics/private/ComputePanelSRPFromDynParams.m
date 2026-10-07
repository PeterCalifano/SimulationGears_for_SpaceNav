function [dAccPanelSRP_IN, dSRPtorque_SCB] = ComputePanelSRPFromDynParams( ...
    dPosSC_IN, dSunPos_IN, dSolarPressure, strDynParams) %#codegen
%% SIGNATURE
% [dAccPanelSRP_IN, dSRPtorque_SCB] = ComputePanelSRPFromDynParams( ...
%     dPosSC_IN, dSunPos_IN, dSolarPressure, strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute panel SRP acceleration and torque from max-fidelity dynamics inputs.
% Resolve attitude, centre of mass and SI units before applying optional
% prepared self-shadowing through visible face areas. Retain each geometric
% face centre for torque; partial illumination does not relocate that centre,
% so partially shadowed torque is approximate. Keep external eclipses in the
% max-fidelity caller.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_IN       (3,1) Spacecraft target-relative inertial position [LU].
% dSunPos_IN      (3,1) Sun target-relative inertial position [LU].
% dSolarPressure  (1,1) Current pressure in the dynamics unit convention.
% strDynParams   Dynamics payload with strSCdata.strSRPpanelData and optional
%                numeric strShadowData in the geometry's declared length unit.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccPanelSRP_IN (3,1) Inertial panel acceleration [LU/s^2].
% dSRPtorque_SCB  (3,1) Body-frame torque about the supplied centre of mass [N m].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026  Pietro Califano, Codex 5.5    Extract max-fidelity panel SRP adapter.
% 04-10-2026  Pietro Califano, Codex GPT-6  Consume prepared self-shadow geometry.
% 06-10-2026  Codex (GPT-6)  Evaluate full-vector/torque truth LUTs in consistent units.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveAttQuat_INfromSCB, ResolveSCCenterOfMass_SCB,
% ResolvePanelSRPUnitsFromDynParams, ComputePanelSrpResponse,
% EvaluateSrpResponseLut, Quat2DCM [MathCore_for_SpaceNav].
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dPosSC_IN (3, 1) double {mustBeFinite}
    dSunPos_IN (3, 1) double {mustBeFinite}
    dSolarPressure (1, 1) double {mustBeFinite, mustBeNonnegative}
    strDynParams (1, 1) struct
end

arguments (Output)
    dAccPanelSRP_IN (3, 1) double
    dSRPtorque_SCB (3, 1) double
end

% Match the normalized attitude used by the shared panel force law.
strPanel = strDynParams.strSCdata.strSRPpanelData;
dQuat_INfromSCB = ResolveAttQuat_INfromSCB(strDynParams);
dQuat_INfromSCB = dQuat_INfromSCB / max(norm(dQuat_INfromSCB), eps);
dCoMpos_SCB = ResolveSCCenterOfMass_SCB(strDynParams);

% Express the Sun direction in the frame of the prepared optical faces.
dSCtoSun_IN = dSunPos_IN - dPosSC_IN;
dDirSCtoSun_IN = dSCtoSun_IN / norm(dSCtoSun_IN);
dDCM_INfromSCB = Quat2DCM(dQuat_INfromSCB);
dDirSCtoSun_SCB = dDCM_INfromSCB.' * dDirSCtoSun_IN;

[dArea, dPressCentre, dCoMpos, dPressureSI, dOutputScale] = ...
    ResolvePanelSRPUnitsFromDynParams(strPanel, dCoMpos_SCB, dSolarPressure, strDynParams);

assert(size(dPressCentre, 2) == size(strPanel.dQuadsNormals_SCB, 2), ...
    'ComputePanelSRPFromDynParams:MissingPressureCenters', ...
    'Panel SRP requires one pressure-centre column per panel.');

% A prepared truth LUT preserves the complete force and body-origin torque model.
if coder.const(isfield(strPanel,'strResponseLut'))
    dSRPtorque_SCB = zeros(3,1);
    if nargout > 1
        [dForce,~,~,dTorque] = EvaluateSrpResponseLut(dDirSCtoSun_SCB,strPanel.strResponseLut,true);
        dSRPtorque_SCB = dPressureSI*(dTorque-cross(dCoMpos,dForce));
    else
        dForce = EvaluateSrpResponseLut(dDirSCtoSun_SCB,strPanel.strResponseLut,true);
    end
    dAccPanelSRP_IN = dOutputScale*dPressureSI/strDynParams.strSCdata.dSCmass * ...
        (dDCM_INfromSCB*dForce);
    return
end

% Evaluate SI panel responses and omit torque work for force-only propagation.
strPanel.dSCquadsArea = dArea;
strPanel.dQuadsPressCentre_SCB = dPressCentre;
dSRPtorque_SCB = zeros(3, 1);
if nargout > 1
    [dForce, ~, dTorqueOrigin] = ComputePanelSrpResponse( ...
        dDirSCtoSun_SCB, strPanel, true, zeros(0, 0), false);
    dSRPtorque_SCB = dPressureSI * (dTorqueOrigin - cross(dCoMpos, dForce));
else
    dForce = ComputePanelSrpResponse(dDirSCtoSun_SCB, strPanel, true);
end
dAccPanelSRP_IN = dOutputScale * dPressureSI / strDynParams.strSCdata.dSCmass * ...
    (dDCM_INfromSCB * dForce);

end
