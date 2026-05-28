function [dDxDt, strAccelInfo] = evalRHS_InertialDynMaxFidelity(dStateTimetag, ...
                                                                 dxState_IN, ...
                                                                 strDynParams, ...
                                                                 strTruthFlags) %#codegen
arguments
    dStateTimetag (1,1) double
    dxState_IN    (:,1) double
    strDynParams  (1,1) struct
    strTruthFlags (1,1) struct = struct()
end
%% SIGNATURE
% [dDxDt, strAccelInfo] = evalRHS_InertialDynMaxFidelity(dStateTimetag, dxState_IN, strDynParams, strTruthFlags)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Simulator-truth inertial orbit RHS. The first six state entries are [r; v] in the inertial frame.
% strDynParams follows the SimulationGears/Nav-System dynamics payload:
%   strMainData       target GM, radius, optional SH/polyhedron gravity, optional attitude ephemeris
%   strBody3rdData    Sun first, Earth second by convention, each with GM and orbit ephemeris
%   strSRPdata        SRP pressure/reference-distance data
%   strSCdata         cannonball data and optional strSRPpanelData
%
% This entry point owns truth-only options while preserving evalRHS_InertialDynOrbit for estimator paths.
% -------------------------------------------------------------------------------------------------------------

assert(numel(dxState_IN) >= 6, ...
    'evalRHS_InertialDynMaxFidelity:InvalidStateSize', ...
    'dxState_IN must contain at least the six inertial orbit states.');

% Extract orbit state handled by shared inertial dynamics kernels.
dxOrbitState = dxState_IN(1:6);

% Resolve truth-model feature flags once before building force-model inputs.
bIncludeMainGravity = GetFlag_(strTruthFlags, 'bIncludeMainGravity', true);
bIncludeSphericalHarmonics = GetFlag_(strTruthFlags, 'bIncludeSphericalHarmonics', true);
bIncludeThirdBodies = GetFlag_(strTruthFlags, 'bIncludeThirdBodies', true);
bIncludeSunThirdBody = GetFlag_(strTruthFlags, 'bIncludeSunThirdBody', bIncludeThirdBodies);
bIncludeEarthThirdBody = GetFlag_(strTruthFlags, 'bIncludeEarthThirdBody', bIncludeThirdBodies);
bIncludeSRP = GetFlag_(strTruthFlags, 'bIncludeSRP', true);
bIncludeEclipse = GetFlag_(strTruthFlags, 'bIncludeEclipse', true);
bUsePanelSRP = GetFlag_(strTruthFlags, 'bUsePanelSRP', true);
bIncludePolyhedronGravity = GetFlag_(strTruthFlags, 'bIncludePolyhedronGravity', true);
bRecomputeSRPpressureFromDistance = ResolveRecomputeSRPFlag_(strDynParams, strTruthFlags);

% Resolve main-body point mass and spherical-harmonic payload.
dMainGM = 0.0;
if bIncludeMainGravity
    dMainGM = strDynParams.strMainData.dGM;
end

dMainCSlmCoeffCols = [];
ui32MaxSHdegree = uint32(0);
if bIncludeSphericalHarmonics && dMainGM > 0.0 && coder.const(isfield(strDynParams.strMainData, 'dSHcoeff')) && ...
        any(abs(strDynParams.strMainData.dSHcoeff) > 0.0, 'all')
    dMainCSlmCoeffCols = strDynParams.strMainData.dSHcoeff;
end
if ~isempty(dMainCSlmCoeffCols) && coder.const(isfield(strDynParams.strMainData, 'ui16MaxSHdegree'))
    ui32MaxSHdegree = uint32(strDynParams.strMainData.ui16MaxSHdegree);
end

% Resolve target attitude and third-body ephemerides required by gravity, SRP, and eclipse.
bHasPolyhedronGravity = bIncludePolyhedronGravity && HasPolyhedronGravityData_(strDynParams);
dDCMmainAtt_INfromTF = ResolveMainAttitude_(dStateTimetag, ...
                                            strDynParams, ...
                                            ~isempty(dMainCSlmCoeffCols) || bHasPolyhedronGravity);
[dBodyEphemerides, d3rdBodiesGM] = ResolveThirdBodyData_(dStateTimetag, ...
                                                         strDynParams, ...
                                                         bIncludeSunThirdBody, ...
                                                         bIncludeEarthThirdBody, ...
                                                         bIncludeThirdBodies, ...
                                                         bIncludeSRP);

% Resolve cannonball SRP coefficient and eclipse state; panel SRP is handled separately.
[dCoeffSRP, dSolarPressure, bHasSunEphemeris] = ResolveCannonballSRP_(dxOrbitState, ...
                                                                       strDynParams, ...
                                                                       dBodyEphemerides, ...
                                                                       bIncludeSRP, ...
                                                                       bRecomputeSRPpressureFromDistance);
bIsInEclipse = false;
if bIncludeSRP && bIncludeEclipse && bHasSunEphemeris
    bIsInEclipse = IsInCylindricalTargetShadow_(dxOrbitState(1:3), ...
                                                dBodyEphemerides(1:3), ...
                                                strDynParams.strMainData.dRefRadius);
end

bHasPanelSRP = bIncludeSRP && bUsePanelSRP && HasPanelSRPData_(strDynParams);
if bHasPanelSRP
    dCoeffForOrbit = [];
else
    dCoeffForOrbit = dCoeffSRP;
end

% Compute polyhedron perturbation as correction over central gravity before shared orbit RHS call.
dAccPolyhedronPert_IN = zeros(3, 1);
if bHasPolyhedronGravity
    dAccPolyhedronPert_IN = ComputePolyhedronGravityCorrection_(dxOrbitState(1:3), ...
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
    [dAccPanelSRP_IN, dSRPtorque_SCB] = ComputePanelSRP_(dxOrbitState(1:3), ...
                                                         dBodyEphemerides(1:3), ...
                                                         dSolarPressure, ...
                                                         strDynParams);
    dDxDt(4:6) = dDxDt(4:6) + dAccPanelSRP_IN;
    bPanelSRPActive = any(abs(dAccPanelSRP_IN) > 0.0);
end

% Return diagnostic acceleration metadata for tests and matching Jacobian logic.
if nargout > 1
    strAccelInfo.dAccPolyhedronPert_IN = dAccPolyhedronPert_IN;
    strAccelInfo.dAccPanelSRP_IN = dAccPanelSRP_IN;
    strAccelInfo.dSRPtorque_SCB = dSRPtorque_SCB;
    strAccelInfo.bPanelSRPActive = bPanelSRPActive;
    strAccelInfo.bCannonballSRPSelected = bIncludeSRP && ~bHasPanelSRP;
    strAccelInfo.bPanelSRPSelected = bHasPanelSRP;
    strAccelInfo.bIsInEclipse = bIsInEclipse;
    strAccelInfo.dSolarPressure = dSolarPressure;
    strAccelInfo.dBodyEphemerides = dBodyEphemerides;
    strAccelInfo.d3rdBodiesGM = d3rdBodiesGM;
end

end

function bFlag = GetFlag_(strFlags, charFieldName, bDefault)
% Return optional truth flag value or caller-provided default.
bFlag = bDefault;
if coder.const(isfield(strFlags, charFieldName))
    bFlag = logical(strFlags.(charFieldName));
end
end

function bRecompute = ResolveRecomputeSRPFlag_(strDynParams, strTruthFlags)
% Resolve SRP pressure-distance scaling precedence from payload then truth flags.
bRecompute = true;
if coder.const(isfield(strDynParams, 'strSRPdata')) && ...
        coder.const(isfield(strDynParams.strSRPdata, 'bRecomputePressureFromDistance'))
    bRecompute = logical(strDynParams.strSRPdata.bRecomputePressureFromDistance);
end
if coder.const(isfield(strTruthFlags, 'bRecomputeSRPpressureFromDistance'))
    bRecompute = logical(strTruthFlags.bRecomputeSRPpressureFromDistance);
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
% Evaluate third-body positions and GM vector according to enabled truth flags.
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
bHasSunEphemeris = ~isempty(dBodyEphemerides) && any(abs(dBodyEphemerides(1:3)) > eps('single'));
if ~bIncludeSRP || ~bHasSunEphemeris || ~coder.const(isfield(strDynParams, 'strSRPdata')) || ...
        ~coder.const(isfield(strDynParams, 'strSCdata'))
    return
end

if bRecomputePressureFromDistance
    dPosSunToSC_IN = dxOrbitState(1:3) - dBodyEphemerides(1:3);
    dDistSunToSC2 = dot(dPosSunToSC_IN, dPosSunToSC_IN);
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

function bHasPolyhedronGravity = HasPolyhedronGravityData_(strDynParams)
% Check whether truth payload carries polyhedron gravity data.
bHasPolyhedronGravity = coder.const(isfield(strDynParams.strMainData, 'strPolyhedronGravityData')) && ...
    ~isempty(strDynParams.strMainData.strPolyhedronGravityData);
end

function bHasPanelSRP = HasPanelSRPData_(strDynParams)
% Check whether truth payload carries panel SRP geometry and optical data.
bHasPanelSRP = coder.const(isfield(strDynParams, 'strSCdata')) && ...
    coder.const(isfield(strDynParams.strSCdata, 'strSRPpanelData')) && ...
    ~isempty(strDynParams.strSCdata.strSRPpanelData);
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

function [dAccPanelSRP_IN, dSRPtorque_SCB] = ComputePanelSRP_(dPosSC_IN, ...
                                                              dSunPos_IN, ...
                                                              dSolarPressure, ...
                                                              strDynParams)
% Compute panelled SRP acceleration and torque from spacecraft-panel data.
strPanel = strDynParams.strSCdata.strSRPpanelData;
dqSCBwrtIN = ResolveSCQuaternion_(strDynParams);
dCoMpos_SCB = ResolveSCCenterOfMass_(strDynParams);

dSCtoSun_IN = dSunPos_IN - dPosSC_IN;
dDirSCtoSun_IN = dSCtoSun_IN / norm(dSCtoSun_IN);
dDCM_INfromSCB = Quat2DCM(dqSCBwrtIN);
dDirSCtoSun_SCB = dDCM_INfromSCB.' * dDirSCtoSun_IN;

[dArea, dPressCentre, dCoMpos, dPressureSI, dOutputScale] = NormalizePanelUnits_(strPanel, ...
                                                                                 dCoMpos_SCB, ...
                                                                                 dSolarPressure, ...
                                                                                 strDynParams);
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

function dAccPolyhedronPert_IN = ComputePolyhedronGravityCorrection_(dPosSC_IN, ...
                                                                     dDCMmainAtt_INfromTF, ...
                                                                     dMainGM, ...
                                                                     strPoly)
% Compute polyhedron gravity perturbation by subtracting central gravity from total polyhedron gravity.
dPosSC_TB = dDCMmainAtt_INfromTF.' * dPosSC_IN;
[dAccPolyhedronTotal_TB, ~] = EvalPolyhedronGrav(dPosSC_TB, ...
                                                 strPoly.ui32FaceVertexIds, ...
                                                 strPoly.dVerticesPos, ...
                                                 strPoly.dDensity, ...
                                                 strPoly.ui32EdgeVertexIds, ...
                                                 strPoly.dEdgeDyadics, ...
                                                 strPoly.dFaceDyadics, ...
                                                 strPoly.dGravConst);

dRadius = norm(dPosSC_TB);
dAccCentral_TB = -dMainGM * dPosSC_TB / dRadius^3;
dAccPolyhedronPert_IN = dDCMmainAtt_INfromTF * (dAccPolyhedronTotal_TB - dAccCentral_TB);
end

function dqSCBwrtIN = ResolveSCQuaternion_(strDynParams)
% Return spacecraft body-to-inertial quaternion, defaulting to identity attitude.
dqSCBwrtIN = [1; 0; 0; 0];
if coder.const(isfield(strDynParams.strSCdata, 'dqSCBwrtIN'))
    dqSCBwrtIN = strDynParams.strSCdata.dqSCBwrtIN(:);
end
end

function dCoMpos_SCB = ResolveSCCenterOfMass_(strDynParams)
% Return spacecraft center of mass in spacecraft body frame, defaulting to origin.
dCoMpos_SCB = zeros(3, 1);
if coder.const(isfield(strDynParams.strSCdata, 'dCoMpos_SCB'))
    dCoMpos_SCB = strDynParams.strSCdata.dCoMpos_SCB(:);
end
end

function [dArea, dPressCentre, dCoMpos, dPressureSI, dOutputScale] = NormalizePanelUnits_(strPanel, ...
                                                                                          dCoMpos_SCB, ...
                                                                                          dSolarPressure, ...
                                                                                          strDynParams)
% Normalize panel geometry, pressure, and acceleration units for ComputeQuadsModelSRP.
charPanelUnit = "m";
if coder.const(isfield(strPanel, 'charLengthUnit'))
    charPanelUnit = string(strPanel.charLengthUnit);
end

dArea = strPanel.dSCquadsArea;
dPressCentre = strPanel.dQuadsPressCentre_SCB;
dCoMpos = dCoMpos_SCB;
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
