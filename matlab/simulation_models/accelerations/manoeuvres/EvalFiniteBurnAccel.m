function [dAccel_IN, dMass, bIsActive] = EvalFiniteBurnAccel(dEvalTime, strBurnData)%#codegen
arguments
    dEvalTime   (1,1) double {mustBeFinite}
    strBurnData (1,1) struct
end
%% PROTOTYPE
% [dAccel_IN, dMass, bIsActive] = EvalFiniteBurnAccel(dEvalTime, strBurnData)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the inertial acceleration and spacecraft mass for a finite-burn profile at one timestamp. The burn is active on [dStartTime, dEndTime) and inactive at the exact end boundary. Acceleration is returned in the same length units selected when the finite-burn profile was built.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dEvalTime:    (1,1) double   Evaluation time [s].
% strBurnData:  (1,1) struct   Finite-burn profile returned by ComputeFiniteBurnFromDeltaV().
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dAccel_IN:    (3,1) double   Inertial thrust acceleration [charLengthUnits/s^2].
% dMass:        (1,1) double   Spacecraft mass at dEvalTime [kg].
% bIsActive:    (1,1) logical  True when dEvalTime is inside the burn interval.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 02-07-2026    Pietro Califano, Codex 5.5      Add deterministic finite-burn acceleration evaluator.
% 02-07-2026    Pietro Califano, Codex 5.5      Use stored length-unit scale for m/km acceleration output.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
dAccel_IN = zeros(3,1);
bIsActive = false;

if strBurnData.dBurnDuration <= 0.0
    dMass = strBurnData.dInitialMass;
    return
end

if dEvalTime < strBurnData.dStartTime
    dMass = strBurnData.dInitialMass;
    return
end

if dEvalTime >= strBurnData.dEndTime
    dMass = strBurnData.dFinalMass;
    return
end

dElapsed = dEvalTime - strBurnData.dStartTime;
dMass = strBurnData.dInitialMass - strBurnData.dMassFlowRate * dElapsed;
dMass = max(dMass, strBurnData.dFinalMass);

% Thrust/mass is SI acceleration [m/s^2]; scale it back to the burn-data
% length unit selected by ComputeFiniteBurnFromDeltaV.
dAccel_IN = strBurnData.dBurnDirection_IN .* (strBurnData.dThrust / dMass / strBurnData.dLengthUnitInMeters);
bIsActive = true;

end
