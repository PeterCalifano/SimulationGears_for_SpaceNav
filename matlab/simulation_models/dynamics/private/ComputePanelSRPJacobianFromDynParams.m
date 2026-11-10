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
% mass and optics fixed, or include a supplied analytical attitude-position tensor.
% Hard quadrature visibility changes have no derivative
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
% 06-10-2026  Codex (GPT-6)  Differentiate the selected truth LUT and its live pressure.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveAttQuat_INfromSCB, ResolvePanelSRPUnitsFromDynParams,
% ComputePanelSrpResponse, EvalJac_SrpResponseLut,
% Quat2DCM [MathCore_for_SpaceNav].
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
dRotation = Quat2DCM(dQuat_INfromSCB);

[dArea, dPressCentre, ~, dPressureSI, dOutputScale] = ...
    ResolvePanelSRPUnitsFromDynParams(strPanel, zeros(3, 1), dSolarPressure, strDynParams);
% Differentiate the selected LUT interpolant and its live inverse-square pressure.
if coder.const(isfield(strPanel,'strResponseLut'))
    [dJacForce,~,~,dForce] = EvalJac_SrpResponseLut( ...
        dRotation.'*dSCtoSun_IN,strPanel.strResponseLut,true);
    dJacPanelSRP_IN = -dOutputScale*dPressureSI/strDynParams.strSCdata.dSCmass * ...
        dRotation*dJacForce*dRotation.';
    if bRecomputePressureFromDistance
        dJacPanelSRP_IN = dJacPanelSRP_IN + ...
            dOutputScale*dPressureSI/strDynParams.strSCdata.dSCmass * ...
            (dRotation*dForce)*(2*dSCtoSun_IN.'/dot(dSCtoSun_IN,dSCtoSun_IN));
    end
    % Direction and force rotation both change under state-dependent pointing.
    dJacPanelSRP_IN = AddAttitudeChain_(dJacPanelSRP_IN, dRotation, ...
        dOutputScale*dPressureSI/strDynParams.strSCdata.dSCmass*dForce, ...
        dOutputScale*dPressureSI/strDynParams.strSCdata.dSCmass*dJacForce, ...
        dSCtoSun_IN, strDynParams.strSCdata);
    return
end
% Reuse the prepared response while freezing visibility on the current branch.
strPanel.dSCquadsArea = dArea;
strPanel.dQuadsPressCentre_SCB = dPressCentre;
[dForce, dResponseJacobian] = ComputePanelSrpResponse(dRotation.'*dSCtoSun_IN, strPanel, true);
dScale = dOutputScale*dPressureSI/strDynParams.strSCdata.dSCmass;
dJacPanelSRP_IN = -dScale*dRotation*dResponseJacobian*dRotation.';
if bRecomputePressureFromDistance
    dJacPanelSRP_IN = dJacPanelSRP_IN + ...
        2*dScale*(dRotation*dForce)*dSCtoSun_IN.'/dot(dSCtoSun_IN,dSCtoSun_IN);
end
dJacPanelSRP_IN = AddAttitudeChain_(dJacPanelSRP_IN, dRotation, ...
    dScale*dForce, dScale*dResponseJacobian, dSCtoSun_IN, strDynParams.strSCdata);

end

function dJacobian = AddAttitudeChain_(dJacobian,dRotation,dBodyAcceleration, ...
    dBodyDirectionJacobian,dScToSun,strSpacecraft)
%% SIGNATURE
% dJacobian = AddAttitudeChain_(dJacobian,dRotation,dBodyAcceleration, ...
%     dBodyDirectionJacobian,dScToSun,strSpacecraft)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply the chain rule of R(r)*a_body(R(r)'*(sun-r)) on the selected smooth branch.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dJacobian              Fixed-attitude acceleration position partials.
% dRotation              Body-to-inertial rotation.
% dBodyAcceleration      Body acceleration at the current pressure.
% dBodyDirectionJacobian Body acceleration partials with respect to the Sun vector.
% dScToSun               Inertial spacecraft-to-Sun vector in dynamics length units.
% strSpacecraft          Optional dJacDCMWrtPos_INfromSCB attitude-position tensor.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacobian              Acceleration position partials including attitude changes.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano  Preserve attitude partials across the prepared SRP merge.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dJacobian (3,3) double
    dRotation (3,3) double
    dBodyAcceleration (3,1) double
    dBodyDirectionJacobian (3,3) double
    dScToSun (3,1) double
    strSpacecraft (1,1) struct
end
arguments (Output)
    dJacobian (3,3) double
end
if coder.const(isfield(strSpacecraft,'dJacDCMWrtPos_INfromSCB'))
    for ui32Axis = uint32(1):uint32(3)
        dRotationPartial = strSpacecraft.dJacDCMWrtPos_INfromSCB(:,:,ui32Axis);
        dJacobian(:,ui32Axis) = dJacobian(:,ui32Axis) + ...
            dRotationPartial*dBodyAcceleration + ...
            dRotation*dBodyDirectionJacobian*(dRotationPartial.'*dScToSun);
    end
end
end
