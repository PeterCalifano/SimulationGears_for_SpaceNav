function dxSTMdt = RHS_2BPwSTM(dTime, xSTMState, dGravParam)%#codegen
arguments (Input)
    dTime       (1,1) double {mustBeNumeric}
    xSTMState   (42,1) double {mustBeNumeric}
    dGravParam  (1,1) double {mustBeNumeric, mustBePositive}
end
arguments (Output)
    dxSTMdt     (42,1) double
end
%% PROTOTYPE
% dxSTMdt = RHS_2BPwSTM(dTime, xSTMState, dGravParam)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Right-hand side of the Cartesian two-body problem augmented with first-order variational equations for the
% state transition matrix (STM). The input vector is [state; vec(STM)] with a 6-state Cartesian orbit model.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTime       (1,1) double   Time tag [TU]. Present for ODE-solver compatibility; dynamics are time-invariant
% xSTMState   (42,1) double  Augmented state [position; velocity; vec(STM)]
% dGravParam  (1,1) double   Central-body gravitational parameter [LU^3/TU^2]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxSTMdt     (42,1) double  Time derivative of augmented state [state derivative; vec(dSTM/dt)]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 31-07-2023    Pietro Califano                 Function reviewed from earlier SGN assignment material.
% 22-04-2026    Pietro Califano, Codex 5.4      Modernized interface, documentation, and naming.
% 22-04-2026    Pietro Califano, Codex 5.4      Inlined state/Jacobian intermediates for wSTM efficiency.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

ui8StateSize = 6;

% Extract Cartesian state and STM from the augmented vector
dPosVelState = xSTMState(1:ui8StateSize);
dSTM = reshape(xSTMState(ui8StateSize+1:end), ui8StateSize, ui8StateSize);

% Extract position and velocity components
dPosX = dPosVelState(1);
dPosY = dPosVelState(2);
dPosZ = dPosVelState(3);
dVelX = dPosVelState(4);
dVelY = dPosVelState(5);
dVelZ = dPosVelState(6);

% Shared inverse-radius terms used by both state and STM dynamics
dRadiusSq = dPosX*dPosX + dPosY*dPosY + dPosZ*dPosZ;
dInvRadius = 1.0 / sqrt(dRadiusSq);
dInvRadius3 = dInvRadius / dRadiusSq;
dInvRadius5 = dInvRadius3 / dRadiusSq;
dGravOverRadius3 = dGravParam * dInvRadius3;
dScale = 3.0 * dGravParam * dInvRadius5;

% State dynamics for the two-body problem
dxdt = [dVelX;
        dVelY;
        dVelZ;
        -dGravOverRadius3 * dPosX;
        -dGravOverRadius3 * dPosY;
        -dGravOverRadius3 * dPosZ];

% Variational equations: dPhi/dt = A(t) * Phi
dUxx = dScale * dPosX * dPosX - dGravOverRadius3;
dUyy = dScale * dPosY * dPosY - dGravOverRadius3;
dUzz = dScale * dPosZ * dPosZ - dGravOverRadius3;
dUxy = dScale * dPosX * dPosY;
dUxz = dScale * dPosX * dPosZ;
dUyz = dScale * dPosY * dPosZ;

dJacobian = [0,    0,    0,    1,  0,  0;
             0,    0,    0,    0,  1,  0;
             0,    0,    0,    0,  0,  1;
             dUxx, dUxy, dUxz, 0,  0,  0;
             dUxy, dUyy, dUyz, 0,  0,  0;
             dUxz, dUyz, dUzz, 0,  0,  0];

dSTMdt = reshape(dJacobian * dSTM, ui8StateSize^2, 1);

% Assemble derivative of the augmented state
dxSTMdt = [dxdt; dSTMdt];

end
