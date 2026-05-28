function dDynMatrix = evalJac_InertialDynMaxFidelity(dStateTimetag, ...
                                                     dxState_IN, ...
                                                     strDynParams, ...
                                                     strTruthFlags, ...
                                                     strAccelInfo) %#codegen
arguments
    dStateTimetag (1,1) double
    dxState_IN    (:,1) double
    strDynParams  (1,1) struct
    strTruthFlags (1,1) struct = struct()
    strAccelInfo  (1,1) struct = struct()
end
%% SIGNATURE
% dDynMatrix = evalJac_InertialDynMaxFidelity(dStateTimetag, dxState_IN, strDynParams, strTruthFlags, strAccelInfo)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical orbit-state Jacobian matching the default truth/reference terms in evalRHS_InertialDynMaxFidelity.
% Panel SRP is intentionally rejected until a matching analytical Jacobian is available.
% -------------------------------------------------------------------------------------------------------------

assert(numel(dxState_IN) >= 6, ...
    'evalJac_InertialDynMaxFidelity:InvalidStateSize', ...
    'dxState_IN must contain at least the six inertial orbit states.');

% Extract orbit state and position used by acceleration partials.
dxOrbitState = dxState_IN(1:6);
dPosSC_IN = dxOrbitState(1:3);

% Resolve truth-model feature flags to match max-fidelity RHS configuration.
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

% Resolve target attitude and third-body ephemerides used by position partials.
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

% Resolve cannonball SRP coefficient and eclipse state; panel SRP Jacobian is unsupported.
[dCoeffSRP, bHasSunEphemeris] = ResolveCannonballSRP_(dxOrbitState, ...
                                                       strDynParams, ...
                                                       dBodyEphemerides, ...
                                                       bIncludeSRP, ...
                                                       bRecomputeSRPpressureFromDistance);
bIsInEclipse = false;
if bIncludeSRP && bIncludeEclipse && bHasSunEphemeris
    bIsInEclipse = IsInCylindricalTargetShadow_(dPosSC_IN, ...
                                                dBodyEphemerides(1:3), ...
                                                strDynParams.strMainData.dRefRadius);
end

bHasPanelSRP = bIncludeSRP && bUsePanelSRP && HasPanelSRPData_(strDynParams);
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
    dPosSunToSC_IN = dPosSC_IN - dBodyEphemerides(1:3);
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
                              bRecomputeSRPpressureFromDistance, ...
                              dSRPdistToSun, ...
                              bIsSRPActive);
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
bHasSunEphemeris = ~isempty(dBodyEphemerides) && any(abs(dBodyEphemerides(1:3)) > eps('single'));
if ~bIncludeSRP || ~bHasSunEphemeris || ~coder.const(isfield(strDynParams, 'strSRPdata')) || ...
        ~coder.const(isfield(strDynParams, 'strSCdata'))
    return
end

if bRecomputePressureFromDistance
    dPosSunToSC_IN = dxOrbitState(1:3) - dBodyEphemerides(1:3);
    dDistSunToSC2 = dot(dPosSunToSC_IN, dPosSunToSC_IN);
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
