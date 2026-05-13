function [dDxDt, strAccelInfo] = computeRefDynAutoDiffFcn(dStateTimetag,...
                                                           objState,...
                                                           strDynParams)%#codegen
arguments
    dStateTimetag   (1,1) {mustBeA(dStateTimetag, "casadi.DM")} 
    objState        (1,1) {mustBeA(objState, ["casadi.MX", "casadi.SX"])} 
    strDynParams    (1,1) {isstruct}
end
%% PROTOTYPE
% [dDxDt, strAccelInfo] = computeRefDynFcn(dStateTimetag,...
%                                           dxState,...
%                                           strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% arguments
%     dStateTimetag   (1,1) {mustBeA(dStateTimetag, "casadi.DM")} 
%     objState        (1,1) {mustBeA(objState, ["casadi.MX", "casadi.sX"])} 
%     strDynParams    (1,1) {isstruct}
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDxDt
% strAccelInfo
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-04-2025    Pietro Califano     Adapted for use with casadi from computeRefDynFcn
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
if not(isfield(strDynParams, 'strBody3rdData'))
    ui32NumOf3rdBodies = 0;
else
    ui32NumOf3rdBodies = length(strDynParams.strBody3rdData);
end

% Check validity of timetags
if not(isempty(strDynParams.strMainData.dSHcoeff))
    if dStateTimetag <= strDynParams.strMainData.strAttData.dTimeLowBound
        dTimeEvalPoint = strDynParams.strMainData.strAttData.dTimeLowBound;

    elseif dStateTimetag >= strDynParams.strMainData.strAttData.dTimeUpBound
        dTimeEvalPoint = strDynParams.strMainData.strAttData.dTimeUpBound;

    else
        dTimeEvalPoint = dStateTimetag;
    end

    % Compute attitude of Main at current time instant
    dDCMmainAtt_INfromTF  = coder.nullcopy(zeros(3, 3));

    dTmpQuat = evalAttQuatChbvPolyWithCoeffs(strDynParams.strMainData.strAttData.ui32PolyDeg, 4, dTimeEvalPoint,...
                                            strDynParams.strMainData.strAttData.dChbvPolycoeffs, ...
                                            strDynParams.strMainData.strAttData.dsignSwitchIntervals, ...
                                            strDynParams.strMainData.strAttData.dTimeLowBound, ...
                                            strDynParams.strMainData.strAttData.dTimeUpBound);

    dDCMmainAtt_INfromTF(1:3, 1:3) = Quat2DCM(dTmpQuat, true);

else
    dDCMmainAtt_INfromTF = zeros(3,3);
end

% Evaluate position Ephemerides of 3rd bodies

dBodyEphemerides = coder.nullcopy(zeros(3*ui32NumOf3rdBodies, 1));
d3rdBodiesGM = coder.nullcopy(zeros(ui32NumOf3rdBodies, 1));

ui32PtrAlloc = 1;

for idB = 1:ui32NumOf3rdBodies

    if dStateTimetag <= strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound
        dTimeEvalPoint = strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound;

    elseif dStateTimetag >= strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound
        dTimeEvalPoint = strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound;
    else
        dTimeEvalPoint = dStateTimetag;
    end

    dBodyEphemerides(ui32PtrAlloc:ui32PtrAlloc+2) = evalChbvPolyWithCoeffs(strDynParams.strBody3rdData(idB).strOrbitData.ui32PolyDeg, ...
                                                                 3, dTimeEvalPoint,...
                                                                 strDynParams.strBody3rdData(idB).strOrbitData.dChbvPolycoeffs, ...
                                                                 strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound, ...
                                                                 strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound);
    
    d3rdBodiesGM(idB) = strDynParams.strBody3rdData(idB).dGM;
    ui32PtrAlloc = ui32PtrAlloc + 3;
end

% Compute SRP coefficient
bRecomputeSRPpressureFromDistance = false;
if isfield(strDynParams.strSRPdata, "bRecomputePressureFromDistance")
    bRecomputeSRPpressureFromDistance = strDynParams.strSRPdata.bRecomputePressureFromDistance;
end

if bRecomputeSRPpressureFromDistance
    assert(isfield(strDynParams.strSRPdata, "dReferenceDistance"), ...
        'computeRefDynAutoDiffFcn:MissingSRPReferenceDistance', ...
        'strDynParams.strSRPdata.dReferenceDistance is required when SRP pressure is recomputed from distance.');
    assert(ui32NumOf3rdBodies > 0, ...
        'computeRefDynAutoDiffFcn:MissingSunEphemerides', ...
        'Sun ephemerides are required when SRP pressure is recomputed from distance.');
    dPosSunToSC_IN = objState(1:3) - dBodyEphemerides(1:3);
    dDistSunToSC2 = sum(dPosSunToSC_IN.^2);
    dPressureSRP = strDynParams.strSRPdata.dP_SRP0 * ...
        (strDynParams.strSRPdata.dReferenceDistance^2 / dDistSunToSC2);
else
    dPressureSRP = strDynParams.strSRPdata.dP_SRP;
end
dCoeffSRP = (dPressureSRP * strDynParams.strSCdata.dReflCoeff * ...
             strDynParams.strSCdata.dA_SRP) / strDynParams.strSCdata.dSCmass; % Move to compute outside, since this

%% Evaluate RHS
% ACHTUNG: Sun must be first in ephemerides and GM data
[dDxDt, strAccelInfo] = evalRHS_InertialDynOrbit(  objState, ...
                                dDCMmainAtt_INfromTF, ...
                                strDynParams.strMainData.dGM, ...
                                strDynParams.strMainData.dRefRadius, ...
                                dCoeffSRP, ...
                                d3rdBodiesGM, ...
                                dBodyEphemerides, ...
                                strDynParams.strMainData.dSHcoeff, ...
                                strDynParams.strMainData.ui16MaxSHdegree, ...
                                [], ...
                                zeros(3, 1), ...
                                false, ...
                                bRecomputeSRPpressureFromDistance);

end
