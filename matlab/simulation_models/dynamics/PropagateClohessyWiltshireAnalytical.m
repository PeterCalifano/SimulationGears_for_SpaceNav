function [dStateAtTime, dSTM] = PropagateClohessyWiltshireAnalytical(dElapsedTime, dMeanAngRate, dInitState) %#codegen
arguments (Input)
    dElapsedTime  (1,1) double {mustBeNumeric}
    dMeanAngRate  (1,1) double {mustBeNumeric, mustBePositive}
    dInitState    (6,1) double {mustBeNumeric}
end
arguments (Output)
    dStateAtTime  (6,1) double
    dSTM          (6,6) double
end
%% PROTOTYPE
% dStateAtTime = PropagateClohessyWiltshireAnalytical(dElapsedTime, dMeanAngRate, dInitState)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical solution of the Clohessy-Wiltshire (CW) linearized relative motion equations from given initial
% conditions. The solution is computed via the CW state transition matrix (STM). Reference frame is LVLH
% (Local Vertical Local Horizontal): x radial, y along-track, z cross-track.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dElapsedTime  (1,1) double   Elapsed time from initial epoch [s]
% dMeanAngRate  (1,1) double   Mean orbital angular rate of the chief [rad/s]
% dInitState    (6,1) double   Initial relative state [x; y; z; vx; vy; vz] in LVLH [km, km/s]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dStateAtTime  (6,1) double   Relative state at dElapsedTime in LVLH [km, km/s]
% dSTM          (6,6) double   State transition matrix at dElapsedTime
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 2022-2023       Pietro Califano                       Initial version.
% 28-03-2026      Pietro Califano, Claude Opus 4.6      Modernized header, arguments block, naming conventions.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Precompute trigonometric and time-dependent quantities
dDeltaAngle      = dMeanAngRate * dElapsedTime;
dCosNt   = cos(dDeltaAngle);
dSinNt   = sin(dDeltaAngle);
dInvN    = 1.0 / dMeanAngRate;

% CW State Transition Matrix (6x6)
dSTM = [4 - 3*dCosNt,            0, 0, dInvN * dSinNt,        2*dInvN * (1 - dCosNt),        0;
        6*(dSinNt - dDeltaAngle),         1, 0, -2*dInvN * (1 - dCosNt), dInvN * (4*dSinNt - 3*dDeltaAngle), 0;
        0,                        0, dCosNt, 0,                  0,                  dInvN * dSinNt;
        3*dMeanAngRate*dSinNt,    0, 0, dCosNt,                  2*dSinNt,                    0;
        -6*dMeanAngRate*(1-dCosNt), 0, 0, -2*dSinNt,            4*dCosNt - 3,                0;
        0,                        0, -dMeanAngRate*dSinNt, 0,   0,                            dCosNt];

% Propagate state through STM
dStateAtTime = dSTM * dInitState;

end
