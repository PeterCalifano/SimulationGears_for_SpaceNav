function dDynMatrix = evalJac_InertialDynMaxFidelity(dStateTimetag, ...
                                                     dxState_IN, ...
                                                     strDynParams, ...
                                                     strModelConfigFlags, ...
                                                     strAccelInfo) %#codegen
arguments
    dStateTimetag (1,1) double
    dxState_IN    (:,1) double
    strDynParams  (1,1) struct
    strModelConfigFlags (1,1) struct = struct()
    strAccelInfo  (1,1) struct = struct()
end
%% PROTOTYPE
% dDynMatrix = evalJac_InertialDynMaxFidelity(dStateTimetag, dxState_IN, strDynParams, strModelConfigFlags, strAccelInfo)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Model-configured max-fidelity inertial orbit-state Jacobian matching evalRHS_InertialDynMaxFidelity.
% Point-mass, third-body, cannonball SRP, and polyhedron partials are analytical; the spherical-harmonics partial
% is the finite-difference target-frame partial returned by EvalJac_ExtSphHarmExpInTargetFrame.
% Panel SRP is intentionally rejected until a matching Jacobian is available.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dStateTimetag:       (1,1) double   Dynamics evaluation time.
% dxState_IN:          (:,1) double   Inertial state; first six entries are Cartesian orbit states.
% strDynParams:        (1,1) struct   Dynamics payload with enabled force-model data.
% strModelConfigFlags: (1,1) struct   Optional compile-time model-configuration overrides.
% strAccelInfo:        (1,1) struct   Optional RHS diagnostic metadata for cached SRP state.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDynMatrix:          (6,6) double   Orbit-state Jacobian matrix.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-05-2026    Pietro Califano, Codex 5.5      Add max-fidelity Jacobian matching the RHS force model.
% 28-05-2026    Pietro Califano, Codex 5.5      Centralize model configuration and document finite-difference SH partial.
% 01-07-2026    Pietro Califano, Codex 5.5      Document zero state partial for time-indexed stochastic acceleration.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveInertialDynMaxFidelityConfig()
% EvalJac_ExtSphHarmExpInTargetFrame()
% EvalJac_CannonballSRP()
% EvalPolyhedronGrav()
% -------------------------------------------------------------------------------------------------------------

%% Function code
assert(numel(dxState_IN) >= 6, ...
    'evalJac_InertialDynMaxFidelity:InvalidStateSize', ...
    'dxState_IN must contain at least the six inertial orbit states.');

% Extract orbit state and position used by acceleration partials.
dxOrbitState = dxState_IN(1:6);
dPosSC_IN = dxOrbitState(1:3);

% Resolve static model configuration to match max-fidelity RHS configuration.
strModelConfig = ResolveInertialDynMaxFidelityConfig(strDynParams, strModelConfigFlags);
dMainGM = 0.0;
if strModelConfig.bIncludeMainGravity
    dMainGM = strDynParams.strMainData.dGM;
end
dMainCSlmCoeffCols = [];
if strModelConfig.bHasSphericalHarmonicsData
    dMainCSlmCoeffCols = strDynParams.strMainData.dSHcoeff;
end
ui32MaxSHdegree = strModelConfig.ui32MaxSHdegree;

% Resolve target attitude and third-body ephemerides used by position partials.
bHasPolyhedronGravity = strModelConfig.bHasPolyhedronGravity;
dDCMmainAtt_INfromTF = ResolveMainAttitude_(dStateTimetag, ...
                                            strDynParams, ...
                                            strModelConfig.bNeedMainAttitude);
[dBodyEphemerides, d3rdBodiesGM] = ResolveThirdBodyData_(dStateTimetag, ...
                                                         strDynParams, ...
                                                         strModelConfig.bIncludeSunThirdBody, ...
                                                         strModelConfig.bIncludeEarthThirdBody, ...
                                                         strModelConfig.bIncludeThirdBodies, ...
                                                         strModelConfig.bIncludeSRP);

% Resolve cannonball SRP coefficient and eclipse state; panel SRP Jacobian is unsupported.
[dCoeffSRP, bHasSunEphemeris] = ResolveCannonballSRP_(dxOrbitState, ...
                                                       strDynParams, ...
                                                       dBodyEphemerides, ...
                                                       strModelConfig.bIncludeSRP, ...
                                                       strModelConfig.bRecomputeSRPpressureFromDistance);
bIsInEclipse = false;
if strModelConfig.bIncludeSRP && strModelConfig.bIncludeEclipse && bHasSunEphemeris
    bIsInEclipse = IsInCylindricalTargetShadow_(dPosSC_IN, ...
                                                dBodyEphemerides(1:3), ...
                                                strDynParams.strMainData.dRefRadius);
end

bHasPanelSRP = strModelConfig.bHasPanelSRP;
assert(~bHasPanelSRP, ...
    'evalJac_InertialDynMaxFidelity:PanelSRPJacobianUnsupported', ...
    'Panel SRP Jacobian is not implemented for the max-fidelity reference dynamics.');

% Initialize state-transition Jacobian with kinematic velocity block.
dDynMatrix = zeros(6, 6);
dDynMatrix(1:3, 4:6) = eye(3);

% Add central point-mass gravity partial.
if dMainGM > 0.0
    dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + EvalPointMassJacobian_(dPosSC_IN, -dMainGM);
end

% Add spherical-harmonic gravity partial transformed from target frame to inertial frame.
if ~isempty(dMainCSlmCoeffCols)
    dPosSC_TB = dDCMmainAtt_INfromTF.' * dPosSC_IN;
    dJacSH_TB = EvalJac_ExtSphHarmExpInTargetFrame(dPosSC_TB, ...
                                                   uint32(ui32MaxSHdegree), ...
                                                   dMainCSlmCoeffCols, ...
                                                   dMainGM, ...
                                                   strDynParams.strMainData.dRefRadius);
    dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
        dDCMmainAtt_INfromTF * dJacSH_TB * dDCMmainAtt_INfromTF.';
end

% Add polyhedron gravity correction partial over central gravity.
if bHasPolyhedronGravity
    dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
        ComputePolyhedronJacobianCorrection_(dPosSC_IN, dDCMmainAtt_INfromTF, dMainGM, ...
                                             strDynParams.strMainData.strPolyhedronGravityData);
end

% Add third-body point-mass partials for enabled bodies.
if ~isempty(dBodyEphemerides)
    ui8NumBodies = uint8(numel(dBodyEphemerides) / 3);
    for idB = 1:double(ui8NumBodies)
        dGMbody = d3rdBodiesGM(idB);
        if dGMbody <= 0.0
            continue;
        end

        idxBody = (3 * (idB - 1) + 1):(3 * idB);
        dPosBodyToSC_IN = dPosSC_IN - dBodyEphemerides(idxBody);
        dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + EvalPointMassJacobian_(dPosBodyToSC_IN, dGMbody);
    end
end

% Add cannonball SRP partial using RHS diagnostic state when available.
if ~isempty(dBodyEphemerides) && ~isempty(dCoeffSRP) && ~bHasPanelSRP
    dPosSunToSC_IN = zeros(3, 1);
    dPosSunToSC_IN(1) = dPosSC_IN(1) - dBodyEphemerides(1);
    dPosSunToSC_IN(2) = dPosSC_IN(2) - dBodyEphemerides(2);
    dPosSunToSC_IN(3) = dPosSC_IN(3) - dBodyEphemerides(3);
    dSRPdistToSun = 0.0;
    bIsSRPActive = false;

    if coder.const(isfield(strAccelInfo, 'dSRPdistToSun'))
        dSRPdistToSun = strAccelInfo.dSRPdistToSun;
    end
    if coder.const(isfield(strAccelInfo, 'bIsSRPActive'))
        bIsSRPActive = strAccelInfo.bIsSRPActive;
    end

    dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
        EvalJac_CannonballSRP(dPosSunToSC_IN, ...
                              dCoeffSRP, ...
                              bIsInEclipse, ...
                              strModelConfig.bRecomputeSRPpressureFromDistance, ...
                              dSRPdistToSun, ...
                              bIsSRPActive);
end

% Stochastic residual acceleration is evaluated from a time-only pre-generated profile, so v1 has no state partial.

end

function dDCMmainAtt_INfromTF = ResolveMainAttitude_(dStateTimetag, strDynParams, bNeedMainAttitude)
% Evaluate target-fixed to inertial attitude only when gravity terms need target frame.
dDCMmainAtt_INfromTF = eye(3);
if ~bNeedMainAttitude || ~coder.const(isfield(strDynParams.strMainData, 'strAttData'))
    return
end

strAttData = strDynParams.strMainData.strAttData;
if ~coder.const(isfield(strAttData, 'dChbvPolycoeffs')) || isempty(strAttData.dChbvPolycoeffs)
    return
end

dTimeEvalPoint = min(max(dStateTimetag, strAttData.dTimeLowBound), strAttData.dTimeUpBound);
dQuat_INfromTF = evalAttQuatChbvPolyWithCoeffs(strAttData.ui32PolyDeg, ...
                                               4, ...
                                               dTimeEvalPoint, ...
                                               strAttData.dChbvPolycoeffs, ...
                                               ResolveSignSwitchIntervals_(strAttData), ...
                                               strAttData.dTimeLowBound, ...
                                               strAttData.dTimeUpBound);
dDCMmainAtt_INfromTF = Quat2DCM(dQuat_INfromTF, true);
end

function dsignSwitchIntervals = ResolveSignSwitchIntervals_(strAttData)
% Return attitude quaternion sign-switch intervals with empty default.
dsignSwitchIntervals = zeros(0, 2);
if coder.const(isfield(strAttData, 'dsignSwitchIntervals'))
    dsignSwitchIntervals = strAttData.dsignSwitchIntervals;
end
end

function [dBodyEphemerides, d3rdBodiesGM] = ResolveThirdBodyData_(dStateTimetag, ...
                                                                  strDynParams, ...
                                                                  bIncludeSunThirdBody, ...
                                                                  bIncludeEarthThirdBody, ...
                                                                  bIncludeThirdBodies, ...
                                                                  bNeedSunEphemeris)
% Evaluate third-body positions and GM vector according to enabled model config flags.
if ~coder.const(isfield(strDynParams, 'strBody3rdData')) || isempty(strDynParams.strBody3rdData)
    dBodyEphemerides = [];
    d3rdBodiesGM = [];
    return
end

ui32NumInputBodies = numel(strDynParams.strBody3rdData);
if ~(bNeedSunEphemeris || bIncludeSunThirdBody || bIncludeEarthThirdBody || bIncludeThirdBodies)
    dBodyEphemerides = [];
    d3rdBodiesGM = [];
    return
end

dBodyEphemerides = zeros(3 * ui32NumInputBodies, 1);
d3rdBodiesGM = zeros(ui32NumInputBodies, 1);

for idB = 1:ui32NumInputBodies
    bIncludeBodyGravity = ShouldIncludeBodyGravity_(idB, ...
                                                    bIncludeSunThirdBody, ...
                                                    bIncludeEarthThirdBody, ...
                                                    bIncludeThirdBodies);
    if idB == 1 && bNeedSunEphemeris
        bIncludePosition = true;
    else
        bIncludePosition = bIncludeBodyGravity;
    end

    idx = (3 * (idB - 1) + 1):(3 * idB);
    if bIncludePosition
        dBodyEphemerides(idx) = EvalBodyOrbitData_(dStateTimetag, strDynParams.strBody3rdData(idB).strOrbitData);
    end
    if bIncludeBodyGravity && coder.const(isfield(strDynParams.strBody3rdData(idB), 'dGM'))
        d3rdBodiesGM(idB) = strDynParams.strBody3rdData(idB).dGM;
    end
end
end

function bIncludeBodyGravity = ShouldIncludeBodyGravity_(idB, bIncludeSun, bIncludeEarth, bIncludeOther)
% Select third-body gravity flag by convention: Sun first, Earth second, others generic.
if idB == 1
    bIncludeBodyGravity = bIncludeSun;
elseif idB == 2
    bIncludeBodyGravity = bIncludeEarth;
else
    bIncludeBodyGravity = bIncludeOther;
end
end

function dPosition_IN = EvalBodyOrbitData_(dStateTimetag, strOrbitData)
% Evaluate Chebyshev body ephemeris with endpoint clamping.
dTimeEvalPoint = min(max(dStateTimetag, strOrbitData.dTimeLowBound), strOrbitData.dTimeUpBound);
dPosition_IN = evalChbvPolyWithCoeffs(strOrbitData.ui32PolyDeg, ...
                                      3, ...
                                      dTimeEvalPoint, ...
                                      strOrbitData.dChbvPolycoeffs, ...
                                      strOrbitData.dTimeLowBound, ...
                                      strOrbitData.dTimeUpBound);
end

function [dCoeffSRP, bHasSunEphemeris] = ResolveCannonballSRP_(dxOrbitState, ...
                                                               strDynParams, ...
                                                               dBodyEphemerides, ...
                                                               bIncludeSRP, ...
                                                               bRecomputePressureFromDistance)
% Compute cannonball SRP coefficient from Sun-spacecraft range.
dCoeffSRP = [];
bHasSunEphemeris = ~isempty(dBodyEphemerides) && norm(dBodyEphemerides(1:3)) > eps('single');

if ~bIncludeSRP || ~bHasSunEphemeris || ~coder.const(isfield(strDynParams, 'strSRPdata')) || ...
        ~coder.const(isfield(strDynParams, 'strSCdata'))
    return
end

if bRecomputePressureFromDistance
    dPosSunToSC_IN = zeros(3, 1);
    dPosSunToSC_IN(1) = dxOrbitState(1) - dBodyEphemerides(1);
    dPosSunToSC_IN(2) = dxOrbitState(2) - dBodyEphemerides(2);
    dPosSunToSC_IN(3) = dxOrbitState(3) - dBodyEphemerides(3);

    dDistSunToSC2 = dPosSunToSC_IN(1)^2 + dPosSunToSC_IN(2)^2 + dPosSunToSC_IN(3)^2;
    assert(dDistSunToSC2 > 0.0, ...
        'evalJac_InertialDynMaxFidelity:ZeroSunSpacecraftDistance', ...
        'Sun-spacecraft distance must be positive when SRP pressure is recomputed from distance.');
    dSolarPressure = strDynParams.strSRPdata.dP_SRP0 * ...
        (strDynParams.strSRPdata.dReferenceDistance^2 / dDistSunToSC2);
else
    dSolarPressure = strDynParams.strSRPdata.dP_SRP;
end

dCoeffSRP = dSolarPressure * strDynParams.strSCdata.dReflCoeff * ...
    strDynParams.strSCdata.dA_SRP / strDynParams.strSCdata.dSCmass;
end

function bIsInEclipse = IsInCylindricalTargetShadow_(dPosSC_IN, dSunPos_IN, dTargetRadius)
% Test cylindrical target shadow using anti-Sun axis and target radius.
bIsInEclipse = false;
if dTargetRadius <= 0.0 || ~any(abs(dSunPos_IN) > 0.0)
    return
end

dSunDir_IN = dSunPos_IN / norm(dSunPos_IN);
dProjectionOnAntiSun = dot(dPosSC_IN, -dSunDir_IN);
if dProjectionOnAntiSun <= 0.0
    return
end

dPerpFromShadowAxis = norm(dPosSC_IN + dProjectionOnAntiSun * dSunDir_IN);
bIsInEclipse = dPerpFromShadowAxis <= dTargetRadius;
end

function dJacPolyCorrection_IN = ComputePolyhedronJacobianCorrection_(dPosSC_IN, ...
                                                                      dDCMmainAtt_INfromTF, ...
                                                                      dMainGM, ...
                                                                      strPoly)
% Compute polyhedron gravity Jacobian correction by subtracting central point-mass partial.
dPosSC_TB = dDCMmainAtt_INfromTF.' * dPosSC_IN;
[~, dJacPolyTotal_TB] = EvalPolyhedronGrav(dPosSC_TB, ...
                                           strPoly.ui32FaceVertexIds, ...
                                           strPoly.dVerticesPos, ...
                                           strPoly.dDensity, ...
                                           strPoly.ui32EdgeVertexIds, ...
                                           strPoly.dEdgeDyadics, ...
                                           strPoly.dFaceDyadics, ...
                                           strPoly.dGravConst);
dJacCentral_TB = EvalPointMassJacobian_(dPosSC_TB, -dMainGM);
dJacPolyCorrection_TB = dJacPolyTotal_TB - dJacCentral_TB;
dJacPolyCorrection_IN = dDCMmainAtt_INfromTF * dJacPolyCorrection_TB * dDCMmainAtt_INfromTF.';
end

function dJac = EvalPointMassJacobian_(dPos, dSignedGM)
% Evaluate point-mass acceleration partial with caller-signed gravitational parameter.
dRadius = norm(dPos);
dInvRadius = 1.0 / dRadius;
dInvRadius3 = dInvRadius^3;
dJac = dSignedGM * (dInvRadius3 * eye(3) - 3.0 * dInvRadius3 * dInvRadius^2 * (dPos * dPos.'));
end
