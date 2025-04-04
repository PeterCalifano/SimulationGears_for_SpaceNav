close all
clear
clc

%% Simple function and jacobian evaluation
import casadi.*
% Define casadi variables
objIn = MX.sym('x', 3, 1);
lambda = @(x) x.^2;
objOut = lambda(objIn);

objFcn = casadi.Function('squared', {objIn}, {objOut});

% Compute symbolic jacobian
objOutJAC = jacobian(objOut, objIn);
objFcnJAC = casadi.Function('Jac', {objIn}, {objOutJAC});

% Evaluate to numerics
dInputValue = ones(size(objIn));

% Evaluate function with numerical input
objFcnOut       = objFcn(dInputValue); % Output is casadi.DM type
objFcnJacOut    = objFcnJAC(dInputValue);

dOutput = objFcnOut(dInputValue);
(objFcnOut);    % Convert to MATLAB double
dOutJac = full(objFcnJacOut); % Convert to MATLAB double

return

%% Computation of flow and STM of ODE  
import casadi.*
% Random Earth orbit used as an example
dTimeGrid = 0:10;
dxState0 = [12000; 5000; 0; 0; 5; 0;];
dGravParam = 398600; % Numeric type
objGravParam = casadi.MX.sym('mu', 1, 1);

dxState = zeros(6, length(dTimeGrid));
dxState(:,1) = dxState0;

% Initialize casadi variable
% ACHTUNG: input symbolic variable must be a char not a string!
objStateFlow_t0tf = casadi.MX.sym('x', size(dxState0, 1), size(dxState0, 2));
objXstate_t0 = objStateFlow_t0tf;

% The naive and inefficient way
for idt = 1:length(dTimeGrid)-1 %#ok<*UNRCH>

    % Apply Euler step to point wise numerical value
    dDeltaStep = dTimeGrid(idt+1) - dTimeGrid(idt);
    % dxState(:, idt+1) = EulerStep(dGravParam, dxState(:, idt), dDeltaStep);

    % Apply Euler integrator symbolically
    objStateFlow_t0tf = EulerStep(objGravParam, objStateFlow_t0tf, dDeltaStep);

end

% Eval result at initial condition to get final state at tf
objFlowFcn = casadi.Function('EulerFlow2BP', {objXstate_t0, objGravParam}, {objStateFlow_t0tf});
objXstateAD = objFlowFcn(dxState0, dGravParam);

% Check difference
dErrNorm = norm(full(objXstateAD - dxState(:,end)));

% Now compute STM t0->tf
objSTM_t0tf             = objStateFlow_t0tf.jacobian(objXstate_t0);
objFlow_t0tf_gravParam  = objStateFlow_t0tf.jacobian(objGravParam);

objSTMfcn_t0tf = casadi.Function('EulerSTM2BP', {objXstate_t0, objGravParam}, {objSTM_t0tf});
dSTM_t0tf = full(objSTMfcn_t0tf(dxState0, dGravParam)); % full() just converts from casadi.DM nume6rical type to MATLAB double

% The most efficient way: using casadi integrator
% TODO
% casadi.Integrator();


%% LOCAL example functions
function dxStateNext = EulerStep(dGravParam, dxState, dDeltaTime) %#ok<*DEFNU>

dxStateNext = dxState + EvalRHS_2BP(0, dxState, dGravParam)*dDeltaTime;

end

function DxDt = EvalRHS_2BP(~, dxState, dGravParam)

    DxDt = [dxState(4); dxState(5); dxState(6);
            -dGravParam/norm(dxState(1:3))^3 * dxState(1:3)];
end
