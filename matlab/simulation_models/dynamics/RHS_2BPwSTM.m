function [dxPhidt] = RHS_2BPwSTM(~, state, mu)

% Derivatives of state vector
dxdt = [state(4:6);
    -mu/norm(state(1:3))^3 * state(1:3)];

n_states = length(dxdt);

% STM at actual time
Phi_t = state(7:end);

Phi_t = reshape(Phi_t, n_states, n_states);

% Jacobian of the dynamics
A_2BP = [0, 0, 0, 1, 0, 0;
 0, 0, 0, 0, 1, 0;
 0, 0, 0, 0, 0, 1;
 (3*mu*state(1)*abs(state(1))*sign(state(1)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2) - mu/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(3/2), (3*mu*state(1)*abs(state(2))*sign(state(2)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2), (3*mu*state(1)*abs(state(3))*sign(state(3)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2), 0, 0, 0;
 (3*mu*state(2)*abs(state(1))*sign(state(1)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2), (3*mu*state(2)*abs(state(2))*sign(state(2)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2) - mu/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(3/2), (3*mu*state(2)*abs(state(3))*sign(state(3)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2), 0, 0, 0;
 (3*mu*state(3)*abs(state(1))*sign(state(1)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2), (3*mu*state(3)*abs(state(2))*sign(state(2)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2), (3*mu*state(3)*abs(state(3))*sign(state(3)))/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(5/2) - mu/(abs(state(1))^2 + abs(state(2))^2 + abs(state(3))^2)^(3/2), 0, 0, 0];

% STM derivatives
dPhidt = A_2BP * Phi_t;
dPhidt = reshape(dPhidt, n_states^2, 1);

% Assembly of derivatives vector for state + STM
dxPhidt = [dxdt; dPhidt];


end