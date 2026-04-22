function dxSTMdt = RHS_2BPwSTM(dTime, xSTMState, dGravParam) %#codegen
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
% 31-07-2023    Pietro Califano     Function reviewed from earlier SGN assignment material.
% 22-04-2026    OpenAI Codex        Modernized interface, documentation, and naming; reused EvalJac_2BP.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalJac_2BP(dState, dGravParam): analytical Jacobian of the Cartesian two-body dynamics
% -------------------------------------------------------------------------------------------------------------

ui8StateSize = 6;

% Extract Cartesian state and STM from the augmented vector
dPosVelState = xSTMState(1:ui8StateSize);
dSTM = reshape(xSTMState(ui8StateSize+1:end), ui8StateSize, ui8StateSize);

% State dynamics for the two-body problem
dPosVec = dPosVelState(1:3);
dPosNorm = norm(dPosVec);
dxdt = [dPosVelState(4:6);
    -dGravParam / dPosNorm^3 * dPosVec];

% Variational equations: dPhi/dt = A(t) * Phi
dJacobian = EvalJac_2BP(dPosVelState, dGravParam);
dSTMdt = reshape(dJacobian * dSTM, ui8StateSize^2, 1);

% Assemble derivative of the augmented state
dxSTMdt = [dxdt; dSTMdt];

end
