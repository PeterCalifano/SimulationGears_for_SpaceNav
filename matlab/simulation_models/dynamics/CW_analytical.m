function x_t = CW_analytical(t, n, x0)
%% PROTOTYPE
% x_t = xCW_analytical(t, n, x0)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Analytical solution of the CW equation from given IC x0 
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% t: [1] elapsed time from t0
% n: [1] mean orbital angular rate [rad/s]
% x0: [6x1] initial condition of the system at time t0
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% x_t: [6x1] state of the system at time t
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CW_STM()
% -------------------------------------------------------------------------------------------------------------
if ~iscolumn(x0)
    x0 = x0';
end

STM = [4-3*cos(n*t), 0, 0, 1/n * sin(n*t), 2/n * (1-cos(n*t)), 0;
    6*(sin(n*t) - n*t), 1, 0, -2/n * (1-cos(n*t)), 1/n *(4*sin(n*t)-3*n*t), 0;
    0, 0, cos(n*t), 0, 0, 1/n * sin(n*t);
    3*n*sin(n*t), 0, 0, cos(n*t), 2*sin(n*t), 0;
    -6*n*(1-cos(n*t)), 0, 0, -2*sin(n*t), 4*cos(n*t)-3, 0;
    0, 0, -n*sin(n*t), 0, 0, cos(n*t)];

% Compute the solution by means of the STM
x_t = STM*x0;

end