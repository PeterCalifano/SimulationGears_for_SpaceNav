close all
clear
clc

%% Script to test Casadi
import casadi.*

% Test map
% 1) Simple creation of symbolic variable
% 2) Test of ODE integration, flow and STM with Euler integrator
% 3) Test of solution flow with casadi using RK5 compatible with casadi
% 4) Test casadi and symbolic toolbox for for computation of Obseration partials
%    and code generation to MEX
% 5) Adapted from merge of 2 and 4: Test RK4 fixed step integrators for the
%    generation of "filters propagation functions" and STM derivation.

%% Test creation of variables and evaluation of functions
objCasadiVar = MX.sym('var', 3, 1);

lambda = @(x) x.^2;

objCasadiVar2 = lambda(objCasadiVar);

%% Test of ODE integration, analytical flow and STM.
% Comparison of time against ode45
% casadi function objects evaluation, DA ODE propagation and comparison
% Import daceypy
%         dacepy = py.importlib.import_module('daceypy');
%         np = py.importlib.import_module('numpy');
%
%         daceypy.DA.init(int8(10), int8(6)); % ORDER 10, 6 VAR
%         rk = dacepy.array.identity(int8(3)) + x0Cart(1:3);
%         vk = dacepy.array.identity(int8(3)) + x0Cart(4:6);

x0Cart = [6135.78313333396
    7141.07348283142
    17168.2510873070
    -0.00827466809510678
    -0.0749154822550970
    -0.154848279483837];
mu = 35.708;

x0 = SX.sym('x', 6, 1);

% Symbolic integrator for analytic Jacobian of the transfer map
tf = 1;
t0 = 0;
tspan = t0:1:tf;
xState = zeros(length(tspan), 6);

% Initialize variables
xState(1, :) = x0Cart;
Flow_t0tf = x0;

FlowHistory = cell(length(tspan), 1);

% Execute integration
for idt = 1:length(tspan)-1

    % Apply Euler step to point wise numerical value
    dt = tspan(idt+1) - tspan(idt);
    xState(idt+1, :) = eulerIntegrator(mu, xState(idt, :)', dt);

    % Apply Euler integrator symbolically
    Flow_t0tf = eulerIntegrator(mu, Flow_t0tf, dt);
    FlowHistory{idt} = Flow_t0tf;

    % Apply Euler integrator using DA
    %             [rk, vk] = eulerIntegratorDA(mu, rk, vk, dt, np);

end

%         t0 = tspan(1); tf = tspan(2);
fx = @(t, x) RHS_2BPtest(t, x, mu);
[tStep, Flow_tNow2tNext] = RKF45symCompatible(fx, x0, 1, t0, tf, 5);

% Get flow from DA integration
%         FlowDA_t0tf = xDA;

% Convert flow object to function
Flow_t0tf_fcn = Function('Flow_t0tf', {x0}, {Flow_t0tf}, {'xSC'}, {'Phi_t0tf'});
Flow_tNow2tNext_fcn = Function('Flow_tNow2tNext', {x0}, {Flow_tNow2tNext}, {'xSC'}, {'Phi_t0tf'});

% Evaluate flow to get solution from initial conditions
fprintf('Final State (casADi Euler): %5.5f \n', full(Flow_t0tf_fcn(x0Cart)));
fprintf('Final State (casADi RK5): %5.5f \n', full(Flow_tNow2tNext_fcn(x0Cart)));
fprintf('Final State (ode): %5.5f \n', xState(end, :));
%         fprintf('Final Position (DA): %5.5f \n', rk.cons());
%         fprintf('Final Velocity (DA): %5.5f \n', vk.cons());

% Compute reference trajectory
STM_t0 = reshape(eye(6), [], 1);
[t, odeOut] = ode45(@(t, x) RHS_2BPwSTM(t, x, mu), tspan,...
    [x0Cart; STM_t0], odeset('RelTol', 1e-12, 'AbsTol', 1e-12, 'MaxStep', 1));

odeOutRV = odeOut(:, 1:6);
odeOutSTM_tf = reshape(odeOut(end, 7:end), 6, 6);

% Derive analytical STM from t0 to tf
STM_t0tf = jacobian(Flow_t0tf, x0);
STM_t0tf_fcn = Function('STM_t0tf', {x0}, {STM_t0tf}, {'xSC'}, {'STM_t0tf'});

STM_RK5_t0tf = jacobian(Flow_tNow2tNext, x0);
STM_RK5_t0tf_fcn = Function('STM_RK5_t0tf', {x0}, {STM_RK5_t0tf}, {'xSC'}, {'STM_RK5_t0tf'});

% Compare DA solution with STM from Variational Equations
%         errSTM_DA = odeOutSTM_tf - STM_t0tf_DA;
%         disp(errSTM_DA);

Nsamples = 1e2;
tic
for id = 1:Nsamples

    xf = full(Flow_t0tf_fcn(x0Cart));
    STM_t0tf = full(STM_t0tf_fcn(x0Cart));

end
avgCasadi_Euler = toc;
avgCasadi_Euler = avgCasadi_Euler/Nsamples;

tic
for id = 1:Nsamples

    xf_RK5 = full(Flow_tNow2tNext_fcn(x0Cart));
    STM_t0tf_RK5 = full(STM_RK5_t0tf_fcn(x0Cart));

end
avgCasadi_RK5 = toc;
avgCasadi_RK5 = avgCasadi_RK5/Nsamples;

tic
for id = 1:Nsamples

    [t, odeOut] = ode45(@(t, x) RHS_2BPwSTM(t, x, mu), [t0, tf],...
        [x0Cart; STM_t0], odeset('RelTol', 1e-8, 'AbsTol', 1e-8));

    odeOutRV = odeOut(end, 1:6);
    STM_t0tf_out = odeOut(end, 7:end);
end

avgOde45 = toc;
avgOde45 = avgOde45/Nsamples;

% Compare Sym with STM from Variational Equations
errState = full(xf) - odeOutRV(end, :)';
errSTM = odeOutSTM_tf - full(STM_t0tf_fcn(x0Cart));
errStateRK5 = full(xf_RK5) - odeOutRV(end, :)';

disp(errState)
disp(errSTM)
disp(errStateRK5)

fprintf('\nCasadi using Euler integrator fcn evaluation took %2.5f [ms] on average\n', avgCasadi_Euler*1000)
fprintf('\nCasadi using RK5 integrator fcn evaluation took %2.5f [ms] on average\n', avgCasadi_RK5*1000)
fprintf('\nOde45 fcn evaluation took %2.5f [ms] on average\n', avgOde45*1000)


% Plot trajectory
%         figure;
%         plot3(xState(:, 1), xState(:, 2), xState(:, 3), 'r-', 'LineWidth', 1.05)
%         hold on
%         plot3(odeOutRV(:, 1), odeOutRV(:, 2), odeOutRV(:, 3), 'b-', 'LineWidth', 1.05);
%         DefaultPlotOpts();
%         xlabel('X');
%         ylabel('Y');
%         zlabel('Z');

return
%% Test of solution flow with casadi using RK5
x0Cart = [6135.78313333396
    7141.07348283142
    17168.2510873070
    -0.00827466809510678
    -0.0749154822550970
    -0.154848279483837];

mu = 35.708;

x0 = SX.sym('x', 6, 1);

% Symbolic integrator for analytic Jacobian of the transfer map
tspan = 0:0.05:5000;
xState = zeros(length(tspan), 6);

% Initialize variables
%         xState(1, :) = x0Cart;
FlowHistory = cell(length(tspan), 1);

% Integrate
fx = @(t, x) RHS_2BPtest(t, x, mu);


tic
% Allocate storage arrays
timeG = zeros(size(tspan));
flowCell = cell(size(tspan));
numflowCell = cell(size(tspan));
numSTMCell = cell(size(tspan));

% Initializa flow variable
Flow_t0tf = x0;
for i = 1:2
    % Extecute one time step
    t0 = tspan(i); tf = tspan(i+1);
    [tGrid, Flow_t0tf] = RKF45symCompatible(fx, Flow_t0tf, 2, t0, tf, 5);
    timeG(i+1) = tGrid(end);
    flowCell{i+1} = Flow_t0tf;
end

toc

% Test of casadi using RKF45 to derive single step propagation function and
% then flow + STM as recursive concatenation

% Derive flow and STM for single time step (propagation function and its jacobian)

t0 = tspan(1); tf = tspan(2);
[tStep, Flow_tNow2tNext] = RKF45symCompatible(fx, x0, 2, t0, tf, 5);
hStep = tStep(2) - tStep(1);
STM_tNow2tNext = jacobian(Flow_tNow2tNext, x0);

% Test for fun :)
HESSIAN = cell(6, 1);
for idC = 1:6
    HESSIAN{idC} = jacobian(STM_tNow2tNext(:, idC), x0);
end

% Convert flow object to function
Flow_t0tf_fcn = Function('Flow_t0tf', {x0}, {Flow_t0tf}, {'xSC'}, {'phi_t0tf'});
Flow_tNow2tNext_fcn = Function('Flow_tNow2tNext', {x0}, {Flow_tNow2tNext}, {'xSC'}, {'phi_tNow2tNext'});
STM_tNow2tNext_fcn = Function('STM_tNow2tNext', {x0}, {STM_tNow2tNext}, {'xSC'}, {'STM_tNow2tNext'});

% NOTE: If it works, the previous scheme can be used for a specialized symbolic integrator
% and it may even be coded to support adaptive size!

% From following reasoning: this may be very useful to compute the
% high order partials, but it is not really useful to speed up
% integrator, at least according to my experiments.

% Construct the trajectory by piecewise evaluation
tic
xk = x0Cart;
STM_k = eye(6);
numSTMCell{1} = STM_k;

for i = 1:length(tspan)-1
    % Extecute one time step of dTime = hStep seconds
    xk = full(Flow_tNow2tNext_fcn(xk));
    STM_k = full(STM_tNow2tNext_fcn(xk));
    STM_t02tf = numSTMCell{i} * STM_k;
    timeG(i+1) = tGrid(end);
    numflowCell{i+1} = xk;
    numSTMCell{i+1} = STM_k;
end
toc


tic
[t, odeOut] = ode45(@(t, x) RHS_2BPwSTM(t, x, mu), tspan,...
    [x0Cart; reshape(eye(6), [], 1)], odeset('RelTol', 1e-13, 'AbsTol', 1e-13));
toc

% Evaluate flow to get solution from initial conditions
fprintf('Final State (casADi): %5.5f \n', full(Flow_t0tf_fcn(x0Cart)));
disp('')
fprintf('Final State (ode): %5.5f \n', odeOut(end, 1:6));
disp('')

err = abs(full(Flow_t0tf_fcn(x0Cart)) - odeOut(end, 1:6)') %#ok<NOPTS>
errLoop = abs(xk - odeOut(end, 1:6)') %#ok<NOPTS>

fprintf('Error wrt ode45 (ode): %5.10f \n', err);
fprintf('Error (LOOP) wrt ode45 (ode): %5.10f \n', errLoop);

errLoopSTM = norm(reshape(STM_t02tf, 36, 1) - odeOut(end, 7:end));
fprintf('STM error wrt ode45 (ode): %5.10f \n', errLoopSTM);

%% Test casadi and symbolic toolbox for for computation of Obseration partials
%         pinholeProject;
%         ExtSHE_AccTB;

% Using Casadi

i_dPosPoint_IN = SX.sym('xFeature', 3, 1);
xState = SX.sym('xState', 7, 1); % [position, quaternion]
i_dKcam = [10560, 0, 502;
    0, 10560, 502;
    0, 0, 1];

tic
[o_dUVpixCoord, o_dDCM_fromINtoCAM] = pinholeProjectSym(i_dKcam, xState(4:7), xState(1:3), i_dPosPoint_IN);

% Observation matrix wrt state vector [r, q]
dUVdxState = jacobian(o_dUVpixCoord, xState);
% Observation matrix wrt feature position [xFeature]
dUVdFeatPos = jacobian(o_dUVpixCoord, i_dPosPoint_IN);

dUVdxState_fcn = Function('dUVdxState_fcn', {xState, i_dPosPoint_IN}, {dUVdxState});
toc

% Using MATLAB symbolic toolbox
xStateMATLAB = sym('xState', [7, 1]);
PosPoint_IN_MATLAB = sym('xFeature', [3, 1]);

tic
[o_dUVpixCoord_MATLAB, o_dDCM_fromINtoCAM_MATLAB] = pinholeProjectSym(i_dKcam, xStateMATLAB(4:7), xStateMATLAB(1:3), PosPoint_IN_MATLAB);
% Observation matrix wrt state vector [r, q]
dUVdxState_MATLAB = jacobian(o_dUVpixCoord_MATLAB, xStateMATLAB);
% Observation matrix wrt feature position [xFeature]
dUVdFeatPos_MATLAB = jacobian(o_dUVpixCoord_MATLAB, PosPoint_IN_MATLAB);

toc

casadiFcnCodegen(dUVdxState_fcn, 'sfcn');

% Test MEX
posTest = [3000; 4323; 3535];
quatTest = [0; 0; -1; 0];
posPointTest = [0.0; 0.0; 0.0];

dUVdxState_fcn_mex([posTest; quatTest], posPointTest)

%         clear dUVdxState_fcn
%         dUVdxState_fcn([posTest; quatTest], posPointTest)
%         dUVdxState_fcn.export_code('matlab','dUVdxState_fcn.m') % Does not work with inputs equal to zero
% % casADi code generation

%
%         x = MX.sym('x', 2);
%         % Test function
%         y = norm(x)^3 + exp(x(1)) + sin(20*x(2)^2);
%         f = Function('fGen', {x}, {y});
%
%         % Simplest and quickest alternative
% %         f.generate('fun.c');
%
%         % More advanced way
%         % Set options in struct() format
%         codegen_options = struct();
%         codegen_options.casadi_real = 'real_T';
%         codegen_options.casadi_int = 'int_T';
%         codegen_options.with_header = true; % Option to generate .h file
%
%         % Define CodeGenerator object
%         % Note: the object requires the function name and the options
%         % struct to be properly defined. (Reference:
%         % https://web.casadi.org/blog/s-function/)
%         codegenObj = CodeGenerator('fGen', codegen_options);
%
%         % Add necessary header for MATLAB API
%         codegenObj.add_include('simstruc.h') % Header as required for S-functions
%         codegenObj.add(f); % Add function target object
%
%         % Generate code
%         mkdir outputCodegen
%         cd outputCodegen
%         codegenObj.generate();
%         cd ..


%% Test RK4 fixed step integrators for the generation of "filters
% propagation functions" and STM derivation.
% Adapted from merge of testcases 2 and 4


% Input values

muValue = 398600;
tStep = 0.5;
tInitial = 0;
tFinal = 1;

x0Cart = [6500;
    0;
    0;
    0;
    sqrt(muValue/6500);
    0];

STM0 = reshape(eye(6), [], 1);
% Propagation function inputs
xState = SX.sym('x', 6, 1);
muBody = SX.sym('mu', 1, 1);
OBSWclock = SX.sym('OBSWclock', 1, 1);



% Stack input signals into vector
inputVec = [xState; muBody];

% EXPERIMENT: this way inputVec has 2 outputs and they can be as
% many as needed.
inputVec = Function('dynRK4propag_1secStep', {xState, muBody}, {inputVec(1:3), inputVec(4:7)}, ...
    {'x', 'mu'}, {'phi_t0tf', 'test1'});

% What Casadi really cares about is the SX variables names.
% Therefore stacking all the inputs in one single vector is doable
% and bring no particular complication. The only additional work to
% do is the correct naming an mapping of the variables in creating
% the functions. Note that the integrator must anyway provide an
% output dxdt consistent with the x state dimension. Put zero for
% the input parameters which are not really propagated.
% Note though: varargin should be usable: because casadi builds the
% Function from the output object of the evaluation. Provided that
% the RHS is written correctly there would be no problem regardless
% of the input state in determining the correct map.
% The problem is in the "middle" time updates of the state, which
% may be effectively solved by indexing and indicating the number
% of states that have a dynamics.

% Dynamics integration for symbolic flow
% TODO: complete and test
fDyn = @(t, x) RHS_2BP(t, x, mu);

% Try the following for fDyn: definition as MATLAB function using
% varargin for all parameters except the state
[tGrid, dynFlow2tFinal] = RK4_autoDiff(fDyn, xState, tStep, tInitial, tFinal);

% STM derivation
STM_1secStep = jacobian(dynFlow2tFinal, xState);

% Package symbolic variables together
dynRK4propag_1secStep = [dynFlow2tFinal; STM_1secStep];

% Function generation (TODO: check options for input/output names)
dynRK4propag_1secStep_obj = Function('dynRK4propag_1secStep', {inputVec}, {dynRK4propag_1secStep}, ...
    {['x', 'mu', 'OBSWclock']}, {'phi_t0tf'});

% Reference values
tic
[t, odeOut] = ode45(@(t, x) RHS_2BPwSTM(t, x, muValue), [tInitial, tFinal],...
    [x0Cart; STM0], odeset('RelTol', 1e-12, 'AbsTol', 1e-12));
toc

finalRef = odeOut(end, :)';


return

% Source code and MEX function generation
casadiFcnCodegen(dUVdxState_fcn, 'mex');






%% LOCAL functions
function xNext = eulerIntegrator(mu, xk, dt)

xNext = xk + RHS_2BPtest(0, xk, mu)*dt;

end

function fx = RHS_2BPtest(~, x, mu)
fx = [x(4); x(5); x(6);
    -mu/norm(x(1:3))^3 * x(1:3)];
end
