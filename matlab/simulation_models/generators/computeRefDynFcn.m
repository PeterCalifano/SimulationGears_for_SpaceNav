function o_dxdt = computeRefDynFcn(i_dStateTimetag,...
                                   i_dxState_IN,...
                                   i_strDynParams)%#codegen
%% PROTOTYPE
% o_dxdt = computeRefDynFcn(i_dStateTimetag,...
%     i_dxState,...
%     i_strDynParams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% Name2                     []
% Name3                     []
% Name4                     []
% Name5                     []
% Name6                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-07-2024      Pietro Califano       Coded from computeDynFcn and FUTURE codes for reference trajectory generation
% 19-07-2024      Pietro Califano       1st version completed and verified
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Check for 3rd bodies
if not(isfield(i_strDynParams, 'strBody3rdData'))
    numOf3rdBodies = 0;
else
    numOf3rdBodies = length(i_strDynParams.strBody3rdData);
end

% Check validity of timetags
if not(isempty(i_strDynParams.strMainData.dSHcoeff))
    if i_dStateTimetag <= i_strDynParams.strMainData.strAttData.dTimeLowBound
        evalPoint = i_strDynParams.strMainData.strAttData.dTimeLowBound;

    elseif i_dStateTimetag >= i_strDynParams.strMainData.strAttData.dTimeUpBound
        evalPoint = i_strDynParams.strMainData.strAttData.dTimeUpBound;

    else
        evalPoint = i_dStateTimetag;
    end

    % Compute attitude of Main at current time instant
    dDCMmainAtt_INfromTF  = coder.nullcopy(zeros(3, 3));

    tmpQuat = evalAttQuatChbvPolyWithCoeffs(i_strDynParams.strMainData.strAttData.ui8PolyDeg, 4, evalPoint,...
        i_strDynParams.strMainData.strAttData.dChbvPolycoeffs, ...
        i_strDynParams.strMainData.strAttData.dsignSwitchIntervals, ...
        i_strDynParams.strMainData.strAttData.dTimeLowBound, ...
        i_strDynParams.strMainData.strAttData.dTimeUpBound);

    dDCMmainAtt_INfromTF(1:3, 1:3) = Quat2DCM(tmpQuat, true);

else
    dDCMmainAtt_INfromTF = zeros(3,3);
end

% Evaluate position Ephemerides of 3rd bodies

dBodyEphemerides = coder.nullcopy(zeros(3*numOf3rdBodies, 1));
d3rdBodiesGM = coder.nullcopy(zeros(numOf3rdBodies, 1));

ptrAlloc = 1;


for idB = 1:numOf3rdBodies

    if i_dStateTimetag <= i_strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound
        evalPoint = i_strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound;

    elseif i_dStateTimetag >= i_strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound
        evalPoint = i_strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound;
    else
        evalPoint = i_dStateTimetag;
    end

    dBodyEphemerides(ptrAlloc:ptrAlloc+2) = evalChbvPolyWithCoeffs(i_strDynParams.strBody3rdData(idB).strOrbitData.ui8PolyDeg, ...
                                                                 3, evalPoint,...
                                                                 i_strDynParams.strBody3rdData(idB).strOrbitData.dChbvPolycoeffs, ...
                                                                 i_strDynParams.strBody3rdData(idB).strOrbitData.dTimeLowBound, ...
                                                                 i_strDynParams.strBody3rdData(idB).strOrbitData.dTimeUpBound);
    
    d3rdBodiesGM(idB) = i_strDynParams.strBody3rdData(idB).dGM;

    ptrAlloc = ptrAlloc + 3;
end

% Compute SRP coefficient
dCoeffSRP = (i_strDynParams.strSRPdata.dP_SRP * i_strDynParams.strSCdata.dReflCoeff * ...
             i_strDynParams.strSCdata.dA_SRP)/i_strDynParams.strSCdata.dSCmass; % Move to compute outside, since this

%% Evaluate RHS
% ACHTUNG: Sun must be first in ephemerides and GM data

o_dxdt = evalRHS_DynOrbit(  i_dxState_IN, ...
                            dDCMmainAtt_INfromTF, ...
                            i_strDynParams.strMainData.dGM, ...
                            i_strDynParams.strMainData.dRefRadius, ...
                            dCoeffSRP, ...
                            d3rdBodiesGM, ...
                            dBodyEphemerides, ...
                            i_strDynParams.strMainData.dSHcoeff, ...
                            i_strDynParams.strMainData.ui16MaxSHdegree);


end
