function [dDxDt, strAccelInfo, dDynMatrix] = ComputeRefDynFcn(dStateTimetag, ...
                                                               dxState, ...
                                                               strDynParams) %#codegen
arguments
    dStateTimetag   (1,1) double {mustBeNumeric, mustBeNonnegative}
    dxState         (:,1) double {mustBeNumeric}
    strDynParams    (1,1) struct
end
%% PROTOTYPE
% [dDxDt, strAccelInfo, dDynMatrix] = ComputeRefDynFcn(dStateTimetag, dxState, strDynParams) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Reference orbit dynamics entry point for simulations and ground-truth propagation. The first six state
% entries are inertial position and velocity. If dxState also carries a 6x6 STM flattened after the state
% (42 states total), the function propagates the STM with the same Jacobian returned as optional output 3.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dStateTimetag   (1,1) double   Evaluation time.
% dxState         (:,1) double   [r; v] or [r; v; Phi(:)].
% strDynParams    (1,1) struct   Dynamics environment assembled by DefineEnvironmentProperties or equivalent.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDxDt           (:,1) double   State derivative, with STM derivative appended when input carries STM.
% strAccelInfo    (1,1) struct   Acceleration components and cached SRP/polyhedron quantities.
% dDynMatrix      (6,6) double   Optional orbit-state Jacobian.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5.5      Upgrade reference dynamics entry point with optional
%                                               Jacobian/STM and polyhedron perturbation support.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% evalRHS_InertialDynOrbit()
% EvalJac_CannonballSRP()
% EvalJac_ExtSphHarmExpInTargetFrame()
% EvalPolyhedronGrav()
% EvalPolyhedronGravPerturbationSamples()
% -------------------------------------------------------------------------------------------------------------

%% Function code
% TODO add pre-mexing of function to speed up (isfield is slow at runtime)

assert(numel(dxState) == 6 || numel(dxState) == 42, ...
    'ComputeRefDynFcn:InvalidStateSize', ...
    'dxState must contain either a 6-state orbit state or a 6-state orbit state plus a flattened 6x6 STM.');

bPropagateSTM = numel(dxState) == 42;
dxOrbitState = dxState(1:6);

if not(isfield(strDynParams, 'strBody3rdData'))
    ui32NumOf3rdBodies = 0;
else
    ui32NumOf3rdBodies = length(strDynParams.strBody3rdData);
end

dMainCSlmCoeffCols = [];
ui32MaxSHdegree = uint32(0);

if isfield(strDynParams.strMainData, 'dSHcoeff')
    dMainCSlmCoeffCols = strDynParams.strMainData.dSHcoeff;
end
if isfield(strDynParams.strMainData, 'ui16MaxSHdegree')
    ui32MaxSHdegree = uint32(strDynParams.strMainData.ui16MaxSHdegree);
end

bHasPolyhedronGravity = coder.const(isfield(strDynParams.strMainData, 'strPolyhedronGravityData') && ...
                            ~isempty(strDynParams.strMainData.strPolyhedronGravityData));

bNeedMainAttitude = coder.const((~isempty(dMainCSlmCoeffCols) || bHasPolyhedronGravity) && ...
                                isfield(strDynParams.strMainData, 'strAttData') && ...
                                isfield(strDynParams.strMainData.strAttData, 'dChbvPolycoeffs') && ...
                                ~isempty(strDynParams.strMainData.strAttData.dChbvPolycoeffs));

% Compute target fixed attitude quaternion from ephemerides
if bNeedMainAttitude

    if dStateTimetag <= strDynParams.strMainData.strAttData.dTimeLowBound
        dTimeEvalPoint = strDynParams.strMainData.strAttData.dTimeLowBound;

    elseif dStateTimetag >= strDynParams.strMainData.strAttData.dTimeUpBound
        dTimeEvalPoint = strDynParams.strMainData.strAttData.dTimeUpBound;

    else
        dTimeEvalPoint = dStateTimetag;
    end

    dTmpQuat = evalAttQuatChbvPolyWithCoeffs(strDynParams.strMainData.strAttData.ui32PolyDeg, 4, dTimeEvalPoint, ...
                                            strDynParams.strMainData.strAttData.dChbvPolycoeffs, ...
                                            strDynParams.strMainData.strAttData.dsignSwitchIntervals, ...
                                            strDynParams.strMainData.strAttData.dTimeLowBound, ...
                                            strDynParams.strMainData.strAttData.dTimeUpBound);

    dDCMmainAtt_INfromTF = Quat2DCM(dTmpQuat, true);
else
    dDCMmainAtt_INfromTF = eye(3);
end

dBodyEphemerides = coder.nullcopy(zeros(3 * ui32NumOf3rdBodies, 1));
d3rdBodiesGM     = coder.nullcopy(zeros(ui32NumOf3rdBodies, 1));

% Compute 3rd body positions from ephemerides
ui32PtrAlloc = 1;
for idB = 1:ui32NumOf3rdBodies

    if dStateTimetag <= strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound
        dTimeEvalPoint = strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound;
    elseif dStateTimetag >= strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound
        dTimeEvalPoint = strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound;
    else
        dTimeEvalPoint = dStateTimetag;
    end

    dBodyEphemerides(ui32PtrAlloc:ui32PtrAlloc+2) = evalChbvPolyWithCoeffs( ...
                                                    strDynParams.strBody3rdData(idB).strOrbitData.ui32PolyDeg, ...
                                                    3, dTimeEvalPoint, ...
                                                    strDynParams.strBody3rdData(idB).strOrbitData.dChbvPolycoeffs, ...
                                                    strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound, ...
                                                    strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound);

    d3rdBodiesGM(idB) = strDynParams.strBody3rdData(idB).dGM;
    ui32PtrAlloc = ui32PtrAlloc + 3;
end

bRecomputeSRPpressureFromDistance = false;
if isfield(strDynParams, 'strSRPdata') && isfield(strDynParams.strSRPdata, "bRecomputePressureFromDistance")
    bRecomputeSRPpressureFromDistance = strDynParams.strSRPdata.bRecomputePressureFromDistance;
end

% Compute SRP parameters and coefficient
dCoeffSRP = [];
if isfield(strDynParams, 'strSRPdata') && isfield(strDynParams, 'strSCdata')

    if bRecomputeSRPpressureFromDistance
        % Force recomputation of SRP pressure from distance
        assert(isfield(strDynParams.strSRPdata, "dReferenceDistance"), ...
            'ComputeRefDynFcn:MissingSRPReferenceDistance', ...
            'strDynParams.strSRPdata.dReferenceDistance is required when SRP pressure is recomputed from distance.');
        assert(ui32NumOf3rdBodies > 0, ...
            'ComputeRefDynFcn:MissingSunEphemerides', ...
            'Sun ephemerides are required when SRP pressure is recomputed from distance.');

        dPosSunToSC_IN = dxOrbitState(1:3) - dBodyEphemerides(1:3);
        dDistSunToSC2 = dot(dPosSunToSC_IN, dPosSunToSC_IN);

        assert(dDistSunToSC2 > 0.0, ...
            'ComputeRefDynFcn:ZeroSunSpacecraftDistance', ...
            'Sun-spacecraft distance must be positive when SRP pressure is recomputed from distance.');

        dPressureSRP = strDynParams.strSRPdata.dP_SRP0 * ...
            (strDynParams.strSRPdata.dReferenceDistance^2 / dDistSunToSC2);
    else
        % Else use input SRP pressure directly
        dPressureSRP = strDynParams.strSRPdata.dP_SRP;
    end

    % Compute SRP coefficient
    dCoeffSRP = (dPressureSRP * strDynParams.strSCdata.dReflCoeff * ...
        strDynParams.strSCdata.dA_SRP) / strDynParams.strSCdata.dSCmass;
end

% Get eclipse status
bIsInEclipse = false;
if coder.const(isfield(strDynParams, 'bIsInEclipse'))
    bIsInEclipse = strDynParams.bIsInEclipse;
end


% TODO modify RHS usage: incorrect usage due to duplication of RHS evaluation within the two functions 
error('Need to update')

[dDxDtOrbit, strAccelInfo] = evalRHS_InertialDynOrbit(dxOrbitState, ...
                                                    dDCMmainAtt_INfromTF, ...
                                                    strDynParams.strMainData.dGM, ...
                                                    strDynParams.strMainData.dRefRadius, ...
                                                    dCoeffSRP, ...
                                                    d3rdBodiesGM, ...
                                                    dBodyEphemerides, ...
                                                    dMainCSlmCoeffCols, ...
                                                    ui32MaxSHdegree, ...
                                                    [], ...
                                                    zeros(3, 1), ...
                                                    bIsInEclipse, ...
                                                    bRecomputeSRPpressureFromDistance);

dAccPolyhedronPert_IN = zeros(3, 1);
if bHasPolyhedronGravity

    strPoly = strDynParams.strMainData.strPolyhedronGravityData;
    dPosSC_TB = dDCMmainAtt_INfromTF.' * dxOrbitState(1:3);

    [~, dAccPolyhedronPert_TB] = EvalPolyhedronGravPerturbationSamples(dPosSC_TB, ...
                                                                        strPoly.ui32FaceVertexIds, ...
                                                                        strPoly.dVerticesPos, ...
                                                                        strPoly.dDensity, ...
                                                                        strPoly.ui32EdgeVertexIds, ...
                                                                        strPoly.dEdgeDyadics, ...
                                                                        strPoly.dFaceDyadics, ...
                                                                        strPoly.dGravConst, ...
                                                                        strPoly.dGravParam);

    dAccPolyhedronPert_IN(:) = dDCMmainAtt_INfromTF * dAccPolyhedronPert_TB(:, 1);
    dDxDtOrbit(4:6) = dDxDtOrbit(4:6) + dAccPolyhedronPert_IN;
end

dDynMatrix = zeros(6, 6);
if nargout > 2 || bPropagateSTM
    dDynMatrix = EvalReferenceDynJacobian_(dxOrbitState, ...
                                        dDCMmainAtt_INfromTF, ...
                                        strDynParams, ...
                                        dCoeffSRP, ...
                                        d3rdBodiesGM, ...
                                        dBodyEphemerides, ...
                                        dMainCSlmCoeffCols, ...
                                        ui32MaxSHdegree, ...
                                        bIsInEclipse, ...
                                        bRecomputeSRPpressureFromDistance, ...
                                        strAccelInfo);
end

if nargout > 1
    strAccelInfo.dAccPolyhedronPert_IN = dAccPolyhedronPert_IN;
end

if bPropagateSTM
    dPhi = reshape(dxState(7:42), 6, 6);
    dDxDt = [dDxDtOrbit; reshape(dDynMatrix * dPhi, 36, 1)];
else
    dDxDt = dDxDtOrbit;
end

end

function dDynMatrix = EvalReferenceDynJacobian_(dxOrbitState, ...
                                                dDCMmainAtt_INfromTF, ...
                                                strDynParams, ...
                                                dCoeffSRP, ...
                                                d3rdBodiesGM, ...
                                                dBodyEphemerides, ...
                                                dMainCSlmCoeffCols, ...
                                                ui32MaxSHdegree, ...
                                                bIsInEclipse, ...
                                                bRecomputeSRPpressureFromDistance, ...
                                                strAccelInfo)
% TODO fix and replace this function with shared helper

dDynMatrix = zeros(6, 6);
dDynMatrix(1:3, 4:6) = eye(3);

dPosSC_IN = dxOrbitState(1:3);
dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
    EvalPointMassJacobian_(dPosSC_IN, -strDynParams.strMainData.dGM);

if ~isempty(dMainCSlmCoeffCols)
    dPosSC_TB = dDCMmainAtt_INfromTF.' * dPosSC_IN;
    dJacSH_TB = EvalJac_ExtSphHarmExpInTargetFrame(dPosSC_TB, ...
        uint32(ui32MaxSHdegree), dMainCSlmCoeffCols, ...
        strDynParams.strMainData.dGM, strDynParams.strMainData.dRefRadius);
    dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
        dDCMmainAtt_INfromTF * dJacSH_TB * dDCMmainAtt_INfromTF.';
end

if isfield(strDynParams.strMainData, 'strPolyhedronGravityData') && ...
        ~isempty(strDynParams.strMainData.strPolyhedronGravityData)

    strPoly = strDynParams.strMainData.strPolyhedronGravityData;
    dPosSC_TB = dDCMmainAtt_INfromTF.' * dPosSC_IN;
    [~, dJacPolyTotal_TB] = EvalPolyhedronGrav(dPosSC_TB, ...
        strPoly.ui32FaceVertexIds, ...
        strPoly.dVerticesPos, ...
        strPoly.dDensity, ...
        strPoly.ui32EdgeVertexIds, ...
        strPoly.dEdgeDyadics, ...
        strPoly.dFaceDyadics, ...
        strPoly.dGravConst);
    dJacPolyPert_TB = dJacPolyTotal_TB + EvalPointMassJacobian_(dPosSC_TB, strPoly.dGravParam);
    dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
        dDCMmainAtt_INfromTF * dJacPolyPert_TB * dDCMmainAtt_INfromTF.';
end

if ~isempty(dBodyEphemerides)
    ui8NumBodies = uint8(numel(dBodyEphemerides) / 3);
    for idB = 1:double(ui8NumBodies)
        dGMbody = d3rdBodiesGM(idB);
        if dGMbody <= 0.0
            continue;
        end
        idxBody = (3 * (idB - 1) + 1):(3 * idB);
        dPosBodyToSC_IN = dPosSC_IN - dBodyEphemerides(idxBody);
        dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
            EvalPointMassJacobian_(dPosBodyToSC_IN, dGMbody);
    end
end

if ~isempty(dBodyEphemerides) && ~isempty(dCoeffSRP)
    dPosSunToSC_IN = dPosSC_IN - dBodyEphemerides(1:3);
    dSRPdistToSun = 0.0;
    bIsSRPActive = false;
    if isfield(strAccelInfo, 'dSRPdistToSun')
        dSRPdistToSun = strAccelInfo.dSRPdistToSun;
    end
    if isfield(strAccelInfo, 'bIsSRPActive')
        bIsSRPActive = strAccelInfo.bIsSRPActive;
    end
    dDynMatrix(4:6, 1:3) = dDynMatrix(4:6, 1:3) + ...
        EvalJac_CannonballSRP(dPosSunToSC_IN, dCoeffSRP, bIsInEclipse, ...
        bRecomputeSRPpressureFromDistance, dSRPdistToSun, bIsSRPActive);
end

end

function dJac = EvalPointMassJacobian_(dPos, dSignedGM)
dRadius = norm(dPos);
dInvRadius = 1.0 / dRadius;
dInvRadius3 = dInvRadius^3;
dJac = dSignedGM * (dInvRadius3 * eye(3) - 3.0 * dInvRadius3 * dInvRadius^2 * (dPos * dPos.'));
end
