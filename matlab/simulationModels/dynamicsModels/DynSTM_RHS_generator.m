close all
clear
clc


%% Dynamics + STM propagation
% NOTE: The generated function input must be adjusted to take one input
% vector instead of multiple variables. Then, just assign the correct values to
% the variables.

%% CR3BP example
%% Parameters definition
mu = 0.012; % Earth Moon Mass ratio for CR3BP

n_states = 6;
syms x [n_states 1] 

%% DYNAMICS Definition
% r1 = ((x(1)+mu)^2 + x(2)^2+ x(3)^2)^0.5
% r2 = ((x(1)+mu-1)^2 + x(2)^2 + x(3)^2)^0.5

U = 0.5*(x(1)^2 + x(2)^2) + (1-mu)/(((x(1) + mu)^2 + x(2)^2+ x(3)^2)^0.5) ...
    + mu/(((x(1) + mu - 1)^2 + x(2)^2 + x(3)^2)^0.5) + 0.5*mu*(1-mu);

% Specify U for planar problem by setting (z, vz) = (0, 0)
U = subs(U, [x(3), x(6)], [0, 0]); 


RHS = [x(4:6);
       2*x(5) + diff(U, x(1));
       -2*x(4) + diff(U, x(2));
       diff(U, x(3))];

RHS([3, 6]) = [];
x([3, 6]) = [];

RHS_handle = matlabFunction(RHS, 'File', 'myRHS');

n_states = length(x);

for i = 1:length(RHS)
    A(:, i) = diff(RHS, x(i));
end

A_handle = matlabFunction(A, 'File', 'myJacobian');
