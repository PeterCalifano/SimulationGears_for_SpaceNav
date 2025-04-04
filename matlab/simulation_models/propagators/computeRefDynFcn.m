function [dDxDt, strAccelInfo] = computeRefDynFcn(dStateTimetag,...
                                                   dxState,...
                                                   strDynParams)%#codegen
arguments
    dStateTimetag   (1,1) {isscalar, isnumeric}
    dxState         (:,1) {isvector, isnumeric}
    strDynParams    (1,1) {isstruct}
end
%% PROTOTYPE
% [dDxDt, strAccelInfo] = computeRefDynFcn(dStateTimetag,...
%                                           dxState,...
%                                           strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% This function computes the reference dynamics for a spacecraft or object in orbit. 
% It evaluates the right-hand side (RHS) of the equations of motion in an inertial frame, considering 
% gravitational forces, solar radiation pressure (SRP), and third-body perturbations if provided. 
% The function also handles the computation of attitude and ephemerides for the main body and third bodies.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% arguments
%     dStateTimetag   (1,1) {isscalar, isnumeric}
%     dxState         (:,1) {isvector, isnumeric}
%     strDynParams    (1,1) {isstruct}
% end
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dDxDt
% % strAccelInfo
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-07-2024    Pietro Califano   Coded from computeDynFcn and FUTURE codes for reference trajectory generation
% 19-07-2024    Pietro Califano   1st version completed and verified
% 12-03-2025    Pietro Califano   Upgrade for use in CScenarioGenerator   
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------

% Check for 3rd bodies
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
dCoeffSRP = (strDynParams.strSRPdata.dP_SRP * strDynParams.strSCdata.dReflCoeff * ...
             strDynParams.strSCdata.dA_SRP) / strDynParams.strSCdata.dSCmass; % Move to compute outside, since this

%% Evaluate RHS
% ACHTUNG: Sun must be first in ephemerides and GM data
[dDxDt, strAccelInfo] = evalRHS_InertialDynOrbit(  dxState, ...
                                dDCMmainAtt_INfromTF, ...
                                strDynParams.strMainData.dGM, ...
                                strDynParams.strMainData.dRefRadius, ...
                                dCoeffSRP, ...
                                d3rdBodiesGM, ...
                                dBodyEphemerides, ...
                                strDynParams.strMainData.dSHcoeff, ...
                                strDynParams.strMainData.ui16MaxSHdegree);


end
