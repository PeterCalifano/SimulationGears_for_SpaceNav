function dxSTMdt = RHS_CR3BPwSTM(dTime, xSTMState, dMassRatio) %#ok<INUSD> %#codegen
arguments (Input)
    dTime       (1,1) double {mustBeNumeric}
    xSTMState   (42,1) double {mustBeNumeric}
    dMassRatio  (1,1) double {mustBeNumeric, mustBePositive, mustBeLessThan(dMassRatio, 1)}
end
arguments (Output)
    dxSTMdt     (42,1) double
end
%% PROTOTYPE
% dxSTMdt = RHS_CR3BPwSTM(dTime, xSTMState, dMassRatio)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Right-hand side of the 3D Circular Restricted Three-Body Problem (CR3BP) augmented with first-order
% variational equations for the state transition matrix (STM). The input vector is [state; vec(STM)] where
% the orbital state is [x; y; z; vx; vy; vz] in the synodic frame.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTime       (1,1) double   Time tag [non-dim]. Present for ODE-solver compatibility; dynamics are autonomous
% xSTMState   (42,1) double  Augmented state [x; y; z; vx; vy; vz; vec(STM)]
% dMassRatio  (1,1) double   Mass ratio m2/(m1+m2) in (0,1)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxSTMdt     (42,1) double  Time derivative of augmented state [state derivative; vec(dSTM/dt)]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 31-07-2023    Pietro Califano     Function reviewed from codes used in
%                                   the SGN 1st assignment (October 2022)
% 22-04-2026    OpenAI Codex        Modernized interface/documentation and inlined shared STM algebra.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

ui8StateSize = 6;

% Extract state vector and STM from input state
dState = xSTMState(1:ui8StateSize);
dSTM = reshape(xSTMState(ui8StateSize+1:end), ui8StateSize, ui8StateSize);

% Extract position and velocity
dPosX = dState(1);
dPosY = dState(2);
dPosZ = dState(3);
dVelX = dState(4);
dVelY = dState(5);
dVelZ = dState(6);

% Mass parameters
dGravP1 = 1 - dMassRatio;
dGravP2 = dMassRatio;

% Relative positions to the two primaries
dRelX_P1 = dPosX + dMassRatio;
dRelX_P2 = dPosX - (1 - dMassRatio);

% Shared inverse-distance powers used by both state and STM dynamics
dDistP1_sq = dRelX_P1^2 + dPosY^2 + dPosZ^2;
dDistP2_sq = dRelX_P2^2 + dPosY^2 + dPosZ^2;
dInvDistP1 = 1.0 / sqrt(dDistP1_sq);
dInvDistP2 = 1.0 / sqrt(dDistP2_sq);
dInvDistP1_3 = dInvDistP1 / dDistP1_sq;
dInvDistP2_3 = dInvDistP2 / dDistP2_sq;
dInvDistP1_5 = dInvDistP1_3 / dDistP1_sq;
dInvDistP2_5 = dInvDistP2_3 / dDistP2_sq;
dGravP1OverDistP1_3 = dGravP1 * dInvDistP1_3;
dGravP2OverDistP2_3 = dGravP2 * dInvDistP2_3;
dScaleP1 = 3.0 * dGravP1 * dInvDistP1_5;
dScaleP2 = 3.0 * dGravP2 * dInvDistP2_5;

% Define RHS for the CR3BP 3D [6x1]
dAccX = dPosX + 2*dVelY - dGravP1OverDistP1_3 * dRelX_P1 - dGravP2OverDistP2_3 * dRelX_P2;
dAccY = dPosY - 2*dVelX - dGravP1OverDistP1_3 * dPosY    - dGravP2OverDistP2_3 * dPosY;
dAccZ =                  - dGravP1OverDistP1_3 * dPosZ    - dGravP2OverDistP2_3 * dPosZ;

dxdt = [dVelX;
        dVelY;
        dVelZ;
        dAccX;
        dAccY;
        dAccZ];

% Define A(t) matrix (partial derivatives of f over state vector)
dSumInvCube = dGravP1OverDistP1_3 + dGravP2OverDistP2_3;
dUxx = 1 - dSumInvCube + dScaleP1 * dRelX_P1^2 + dScaleP2 * dRelX_P2^2;
dUyy = 1 - dSumInvCube + dScaleP1 * dPosY^2    + dScaleP2 * dPosY^2;
dUzz =   - dSumInvCube + dScaleP1 * dPosZ^2    + dScaleP2 * dPosZ^2;
dUxy = dScaleP1 * dRelX_P1 * dPosY + dScaleP2 * dRelX_P2 * dPosY;
dUxz = dScaleP1 * dRelX_P1 * dPosZ + dScaleP2 * dRelX_P2 * dPosZ;
dUyz = dScaleP1 * dPosY    * dPosZ + dScaleP2 * dPosY    * dPosZ;

dJacobian = [0,    0,    0,    1,  0,  0;
             0,    0,    0,    0,  1,  0;
             0,    0,    0,    0,  0,  1;
             dUxx, dUxy, dUxz, 0,  2,  0;
             dUxy, dUyy, dUyz, -2, 0,  0;
             dUxz, dUyz, dUzz, 0,  0,  0];

dSTMdt = reshape(dJacobian * dSTM, ui8StateSize^2, 1);

% Assembly dstate + dSTM
dxSTMdt = [dxdt; dSTMdt];

end
