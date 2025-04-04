close all
clear
clc


import casadi.*

% TEST SETUP
% Define casadi variables
objIn = MX.sym('x', 3, 1);
lambda = @(x) x.^2;
objOut = lambda(objIn);

objFcn = casadi.Function('squared', {objIn}, {objOut});

% Compute symbolic jacobian
objFcnJAC = objFcn.jacobian();

% Evaluate to numerics
dInputValue = ones(size(objIn));

objFcnOut = objFcn(dInputValue); % Output is casadi.DM type

dOutput = full(objFcnOut); % Convert to mATLAB double

% Automatic conversion of a struct from MATLAB primitives to casadi types
