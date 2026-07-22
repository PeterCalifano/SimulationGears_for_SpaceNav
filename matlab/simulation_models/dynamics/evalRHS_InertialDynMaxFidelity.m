function [dDxDt, strAccelInfo] = evalRHS_InertialDynMaxFidelity(dStateTimetag, ...
                                                                 dxState_IN, ...
                                                                 strDynParams, ...
                                                                 strModelConfigFlags) %#codegen
arguments
    dStateTimetag (1,1) double
    dxState_IN    (:,1) double
    strDynParams  (1,1) struct
    strModelConfigFlags (1,1) struct = struct()
end
%% PROTOTYPE
% [dDxDt, strAccelInfo] = evalRHS_InertialDynMaxFidelity(dStateTimetag, dxState_IN, strDynParams, strModelConfigFlags)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Model-configured max-fidelity inertial orbit RHS. The first six state entries are [r; v] in the inertial frame.
% strDynParams follows the SimulationGears/Nav-System dynamics payload:
%   strMainData       target GM, radius, optional SH/polyhedron gravity, optional attitude ephemeris
%   strBody3rdData    Sun first, Earth second by convention, each with GM and orbit ephemeris
%   strSRPdata        SRP pressure/reference-distance data
%   strSCdata         cannonball data and optional strSRPpanelData
%
% This entry point owns compile-time model-configuration options while preserving evalRHS_InertialDynOrbit for
% estimator paths. Target gravity is exclusive and reported by ui8SelectedGravityModel: 0 none, 1 central,
% 2 spherical harmonics, and 3 polyhedron.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dStateTimetag:       (1,1) double   Dynamics evaluation time.
% dxState_IN:          (:,1) double   Inertial state; first six entries are Cartesian orbit states.
% strDynParams:        (1,1) struct   Dynamics payload with enabled force-model data.
% strModelConfigFlags: (1,1) struct   Optional compile-time model-configuration overrides.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDxDt:               (:,1) double   State derivative for the inertial orbit state.
% strAccelInfo:        (1,1) struct   Acceleration diagnostics, including the selected target-gravity model ID.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-05-2026    Pietro Califano, Codex 5.5      Add max-fidelity RHS wrapper around shared orbit dynamics.
% 28-05-2026    Pietro Califano, Codex 5.5      Centralize model configuration and schema-driven SH activation.
% 01-07-2026    Pietro Califano, Codex 5.5      Add optional pre-generated stochastic residual acceleration hook.
% 22-07-2026    Pietro Califano, Codex           Evaluate exactly one selected target-gravity model.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveInertialDynMaxFidelityConfig()
% evalRHS_InertialDynOrbit()
% EvalGaussMarkovAccel()
% IsInCylindricalTargetShadow()
% ComputePolyhedronGravityCorrection()
% ComputePanelSRPFromDynParams()
% -------------------------------------------------------------------------------------------------------------

%% Function code
assert(numel(dxState_IN) >= 6, ...
    'evalRHS_InertialDynMaxFidelity:InvalidStateSize', ...
    'dxState_IN must contain at least the six inertial orbit states.');

% Extract orbit state handled by shared inertial dynamics kernels.
dxOrbitState = dxState_IN(1:6);

% Resolve static model configuration once before building force-model inputs.
strModelConfig = ResolveInertialDynMaxFidelityConfig(strDynParams, strModelConfigFlags);
ui8SelectedGravityModel = strModelConfig.ui8SelectedGravityModel;
dMainGM = 0.0;
if strModelConfig.bIncludeMainGravity
    dMainGM = strDynParams.strMainData.dGM;
end
dMainCSlmCoeffCols = [];
if ui8SelectedGravityModel == uint8(2)
    dMainCSlmCoeffCols = strDynParams.strMainData.dSHcoeff;
end
ui32MaxSHdegree = strModelConfig.ui32MaxSHdegree;

% Resolve target attitude and third-body ephemerides required by gravity, SRP, and eclipse.
dDCMmainAtt_INfromTF = ResolveMainAttitude_(dStateTimetag, ...
                                            strDynParams, ...
                                            strModelConfig.bNeedMainAttitude);
[dBodyEphemerides, d3rdBodiesGM] = ResolveThirdBodyData_(dStateTimetag, ...
                                                         strDynParams, ...
                                                         strModelConfig.bIncludeSunThirdBody, ...
                                                         strModelConfig.bIncludeEarthThirdBody, ...
                                                         strModelConfig.bIncludeThirdBodies, ...
                                                         strModelConfig.bIncludeSRP);

% Resolve cannonball SRP coefficient and eclipse state; panel SRP is handled separately.
[dCoeffSRP, dSolarPressure, bHasSunEphemeris] = ResolveCannonballSRP_(dxOrbitState, ...
                                                                       strDynParams, ...
                                                                       dBodyEphemerides, ...
                                                                       strModelConfig.bIncludeSRP, ...
                                                                       strModelConfig.bRecomputeSRPpressureFromDistance);
bIsInEclipse = false;
if strModelConfig.bIncludeSRP && strModelConfig.bIncludeEclipse && bHasSunEphemeris
    bIsInEclipse = IsInCylindricalTargetShadow(dxOrbitState(1:3), ...
                                               dBodyEphemerides(1:3), ...
                                               strDynParams.strMainData.dRefRadius);
end

bHasPanelSRP = strModelConfig.bHasPanelSRP;
if bHasPanelSRP
    dCoeffForOrbit = [];
else
    dCoeffForOrbit = dCoeffSRP;
end

% Compute polyhedron perturbation as correction over central gravity before shared orbit RHS call.
dAccPolyhedronPert_IN = zeros(3, 1);
if ui8SelectedGravityModel == uint8(3)
    dAccPolyhedronPert_IN = ComputePolyhedronGravityCorrection(dxOrbitState(1:3), ...
                                                               dDCMmainAtt_INfromTF, ...
                                                               dMainGM, ...
                                                               strDynParams.strMainData.strPolyhedronGravityData);
end

% Evaluate shared inertial orbit RHS for point mass, SH, third bodies, cannonball SRP, and external acceleration.
[dDxDt, strAccelInfo] = evalRHS_InertialDynOrbit(dxOrbitState, ...
                                                 dDCMmainAtt_INfromTF, ...
                                                 dMainGM, ...
                                                 strDynParams.strMainData.dRefRadius, ...
                                                 dCoeffForOrbit, ...
                                                 d3rdBodiesGM, ...
                                                 dBodyEphemerides, ...
                                                 dMainCSlmCoeffCols, ...
                                                 ui32MaxSHdegree, ...
                                                 [], ...
                                                 dAccPolyhedronPert_IN, ...
                                                 bIsInEclipse);

% Add panel SRP acceleration and torque when truth model has panel geometry and sunlight.
dAccPanelSRP_IN = zeros(3, 1);
dSRPtorque_SCB = zeros(3, 1);
bPanelSRPActive = false;
if bHasPanelSRP && bHasSunEphemeris && ~bIsInEclipse
    [dAccPanelSRP_IN, dSRPtorque_SCB] = ComputePanelSRPFromDynParams(dxOrbitState(1:3), ...
                                                                     dBodyEphemerides(1:3), ...
                                                                     dSolarPressure, ...
                                                                     strDynParams);
    dDxDt(4:6) = dDxDt(4:6) + dAccPanelSRP_IN;
    bPanelSRPActive = any(abs(dAccPanelSRP_IN) > 0.0);
end

% Add truth-only stochastic residual acceleration when a pre-generated profile is present.
dAccStochastic_IN = zeros(3, 1);
if strModelConfig.bHasStochasticAccelData
    dAccStochastic_IN = EvalGaussMarkovAccel(dStateTimetag, strDynParams.strStochasticAccelData);
    dDxDt(4:6) = dDxDt(4:6) + dAccStochastic_IN;
end

% Return diagnostic acceleration metadata for tests and matching Jacobian logic.
if nargout > 1
    strAccelInfo.dAccPolyhedronPert_IN = dAccPolyhedronPert_IN;
    strAccelInfo.dAccPanelSRP_IN = dAccPanelSRP_IN;
    strAccelInfo.dAccStochastic_IN = dAccStochastic_IN;
    strAccelInfo.ui8SelectedGravityModel = ui8SelectedGravityModel;
    strAccelInfo.dSRPtorque_SCB = dSRPtorque_SCB;
    strAccelInfo.bPanelSRPActive = bPanelSRPActive;
    strAccelInfo.bCannonballSRPSelected = strModelConfig.bIncludeSRP && ~bHasPanelSRP;
    strAccelInfo.bPanelSRPSelected = bHasPanelSRP;
    strAccelInfo.bIsInEclipse = bIsInEclipse;
    strAccelInfo.dSolarPressure = dSolarPressure;
    strAccelInfo.dBodyEphemerides = dBodyEphemerides;
    strAccelInfo.d3rdBodiesGM = d3rdBodiesGM;
end

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
ui32NumOutputBodies = ui32NumInputBodies;

if ~(bNeedSunEphemeris || bIncludeSunThirdBody || bIncludeEarthThirdBody || bIncludeThirdBodies)
    dBodyEphemerides = [];
    d3rdBodiesGM = [];
    return
end

dBodyEphemerides = zeros(3 * ui32NumOutputBodies, 1);
d3rdBodiesGM = zeros(ui32NumOutputBodies, 1);

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

function [dCoeffSRP, dSolarPressure, bHasSunEphemeris] = ResolveCannonballSRP_(dxOrbitState, ...
                                                                               strDynParams, ...
                                                                               dBodyEphemerides, ...
                                                                               bIncludeSRP, ...
                                                                               bRecomputePressureFromDistance)
% Compute cannonball SRP coefficient and solar pressure from Sun-spacecraft range.
dCoeffSRP = [];
dSolarPressure = 0.0;
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
        'evalRHS_InertialDynMaxFidelity:ZeroSunSpacecraftDistance', ...
        'Sun-spacecraft distance must be positive when SRP pressure is recomputed from distance.');
    dSolarPressure = strDynParams.strSRPdata.dP_SRP0 * ...
        (strDynParams.strSRPdata.dReferenceDistance^2 / dDistSunToSC2);
else
    dSolarPressure = strDynParams.strSRPdata.dP_SRP;
end

dCoeffSRP = dSolarPressure * strDynParams.strSCdata.dReflCoeff * ...
    strDynParams.strSCdata.dA_SRP / strDynParams.strSCdata.dSCmass;
end
