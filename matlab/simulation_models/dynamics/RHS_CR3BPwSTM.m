function [dxSTMdt] = RHS_CR3BPwSTM(time, xSTMState, mu) %#ok<INUSD> %#codegen
%% PROTOTYPE
% [dxSTMdt] = RHS_CR3BPwSTM(~, xSTMState, mu)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function evaluating the RHS of the CR3BP of both dynamics and Variational
% equations for the STM (1st order) given a value of mu (mass ratio of the
% CR3BP model).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% time: [1]
% xSTMState: [46x1]
% mu: [1]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxSTMdt: [46x1]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 31-07-2023    Pietro Califano     Function reviewed from codes used in
%                                   the SGN 1st assignment (October 2022)
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RHS_CR3BP(xState, mu): Function computing the RHS of the CR3BP
% J_CR3BP(STM_State, mu): Function computing the analytical Jacobian of the CR3BP
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades


%% Function code

% Extract state vector and STM from input state
xState = xSTMState(1:6);
STM_State = xSTMState(7:end);

% Define RHS for the CR3BP 3D [6x1]
dxdt = RHS_CR3BP(xState, mu);

% Define A(t) matrix (partial derivatives of f over state vector)
AJ = J_CR3BP(STM_State, mu);

% Compute derivative of the STM (matrix operation)
state_size = length(AJ);
dPhidt_mat = AJ*reshape(STM_State, state_size, state_size);
dPhidt = reshape(dPhidt_mat, state_size^2, 1);

% Assembly dstate + dSTM
dxSTMdt = [dxdt; dPhidt];

end