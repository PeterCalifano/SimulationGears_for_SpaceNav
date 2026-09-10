function [objStateHistory, dTimeGrid, strStatistics] = ...
    PropagateAdaptiveStepAutoDiff( ...
    fcnStateDerivative, dTimeSpan, objInitialState, dInitialStep, ...
    strDynParams, strModelConfigFlags, settings)
%% SIGNATURE
% [objStateHistory, dTimeGrid, strStatistics] = ...
%     PropagateAdaptiveStepAutoDiff( ...
%     fcnStateDerivative, dTimeSpan, objInitialState, dInitialStep, ...
%     strDynParams, strModelConfigFlags, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Propagate a numerical or automatic-differentiation state through one
% adaptive interval boundary. Finite double states reuse SimulationGears'
% canonical RKF45 provider. CasADi MX states build a differentiable CVODES
% flow evaluated with adaptive internal steps and returned on a deterministic
% output grid whose final interval is shortened to the exact endpoint.
%
% CasADi SX is supported by the explicit RK4/RK8 AutoDiff step functions but
% not by this adaptive interval provider because the CVODES Function cannot be
% evaluated symbolically as SX in CasADi 3.6.7. Use an MX initial state when
% constructing an adaptive symbolic flow.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnStateDerivative              (1,1) function_handle using the standard four-input RHS contract
% dTimeSpan                       (1,2) finite double start and end timestamps
% objInitialState                 (Nx1) finite double or CasADi MX initial state
% dInitialStep                    (1,1) finite positive initial step magnitude
% strDynParams                    (1,1) struct runtime dynamics payload forwarded to RHS
% strModelConfigFlags             (1,1) struct model selection forwarded to RHS
% settings.dRelativeTolerance     (1,1) positive double relative tolerance
% settings.dAbsoluteTolerance     (1,1) positive double absolute tolerance
% settings.dMinimumStep           (1,1) nonnegative double minimum internal step
% settings.dMaximumStep           (1,1) positive double maximum internal/output step
% settings.ui32MaximumAttempts    (1,1) uint32 maximum internal step count
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objStateHistory     (MxN) double or CasADi MX state history, one state per row
% dTimeGrid           (Mx1) double timestamps corresponding to history rows
% strStatistics       (1,1) numerical propagation statistics for double input;
%                     empty struct for an unevaluated CasADi symbolic graph
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-07-2026  Pietro Califano, Codex     Restore numerical and symbolic adaptive propagation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% PropagateAdaptiveStep; optional CasADi 3.6.7 for symbolic MX input
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    fcnStateDerivative (1,1) function_handle
    dTimeSpan (1,2) double {mustBeFinite}
    objInitialState (:,1)
    dInitialStep (1,1) double {mustBeFinite, mustBePositive}
    strDynParams (1,1) struct
    strModelConfigFlags (1,1) struct
    settings.dRelativeTolerance (1,1) double ...
        {mustBeFinite, mustBePositive} = 1.0e-8
    settings.dAbsoluteTolerance (1,1) double ...
        {mustBeFinite, mustBePositive} = 1.0e-10
    settings.dMinimumStep (1,1) double ...
        {mustBeFinite, mustBeNonnegative} = 0.0
    settings.dMaximumStep (1,1) double ...
        {mustBeFinite, mustBePositive} = realmax
    settings.ui32MaximumAttempts (1,1) uint32 ...
        {mustBePositive} = uint32(1000000)
end

arguments (Output)
    objStateHistory (:,:)
    dTimeGrid (:,1) double
    strStatistics
end

if isa(objInitialState, 'double')
    % Keep numerical RKF45 behavior identical to the canonical provider and
    % independent of CasADi installation or path state.
    [objStateHistory, dTimeGrid, strStatistics] = ...
        PropagateAdaptiveStep( ...
        fcnStateDerivative, dTimeSpan, objInitialState, dInitialStep, ...
        strDynParams, strModelConfigFlags, ...
        dRelativeTolerance=settings.dRelativeTolerance, ...
        dAbsoluteTolerance=settings.dAbsoluteTolerance, ...
        dMinimumStep=settings.dMinimumStep, ...
        dMaximumStep=settings.dMaximumStep, ...
        ui32MaximumAttempts=settings.ui32MaximumAttempts);
    return;
end

ValidateSymbolicState_(objInitialState);
if settings.dMinimumStep > settings.dMaximumStep
    error('PropagateAdaptiveStepAutoDiff:InvalidStepBounds', ...
        'The minimum step cannot exceed the maximum step.');
end

dDuration = dTimeSpan(2) - dTimeSpan(1);
if dDuration == 0.0
    objStateHistory = objInitialState.';
    dTimeGrid = dTimeSpan(1);
    strStatistics = struct([]);
    return;
end

% Use deterministic output checkpoints while CVODES controls its internal
% accepted and rejected steps when the symbolic graph is evaluated.
dOutputStep = min(dInitialStep, settings.dMaximumStep);
dTimeGrid = BuildTimeGrid_(dTimeSpan, dOutputStep);
dOutputOffsets = abs(dTimeGrid(2:end) - dTimeSpan(1)).';
dDirection = sign(dDuration);
dDurationMagnitude = abs(dDuration);

% Build a physical-time RHS on a positive integration horizon so the same
% CasADi configuration supports forward and backward propagation.
objIntegrationTime = casadi.MX.sym('objIntegrationTime', 1, 1);
objIntegrationState = casadi.MX.sym( ...
    'objIntegrationState', size(objInitialState, 1), 1);
objPhysicalTime = dTimeSpan(1) + dDirection * objIntegrationTime;
objStateDerivative = dDirection * fcnStateDerivative( ...
    objPhysicalTime, objIntegrationState, ...
    strDynParams, strModelConfigFlags);
ValidateDerivative_(objStateDerivative, objIntegrationState);

strDae = struct( ...
    'x', objIntegrationState, ...
    't', objIntegrationTime, ...
    'ode', objStateDerivative);
strCasadiOptions = struct( ...
    'reltol', settings.dRelativeTolerance, ...
    'abstol', settings.dAbsoluteTolerance, ...
    'step0', min([dInitialStep, settings.dMaximumStep, ...
        dDurationMagnitude]), ...
    'min_step_size', settings.dMinimumStep, ...
    'max_step_size', min(settings.dMaximumStep, dDurationMagnitude), ...
    'max_num_steps', double(settings.ui32MaximumAttempts), ...
    'linear_multistep_method', 'adams');

% CVODES remains behind this optional symbolic branch; merely running
% SetupSimGears or using the numerical branch does not load CasADi.
objIntegrator = casadi.integrator( ...
    'SimulationGearsAdaptiveFlow', 'cvodes', strDae, ...
    0.0, dOutputOffsets, strCasadiOptions);
strIntegrationResult = objIntegrator('x0', objInitialState);
objStateHistory = [objInitialState, strIntegrationResult.xf].';

% Internal integration counts are produced only when the returned symbolic
% graph is numerically evaluated, so construction-time statistics are empty.
strStatistics = struct([]);

end

function ValidateSymbolicState_(objInitialState)
% Require MX for the adaptive graph while retaining explicit SX diagnostics.
if isa(objInitialState, 'casadi.SX')
    error('PropagateAdaptiveStepAutoDiff:SXAdaptiveUnsupported', ...
        ['CasADi 3.6.7 CVODES does not support symbolic SX evaluation. ', ...
         'Use a CasADi MX initial state for adaptive propagation.']);
end
if ~isa(objInitialState, 'casadi.MX')
    error('PropagateAdaptiveStepAutoDiff:InvalidStateType', ...
        'The initial state must be a finite double column or a CasADi MX column.');
end
if isempty(objInitialState)
    error('PropagateAdaptiveStepAutoDiff:EmptyInitialState', ...
        'The initial state must contain at least one element.');
end
end

function dTimeGrid = BuildTimeGrid_(dTimeSpan, dMaximumOutputStep)
% Build exact signed output checkpoints with a shortened final interval.
dDuration = dTimeSpan(2) - dTimeSpan(1);
dDirection = sign(dDuration);
ui32StepCount = uint32(ceil(abs(dDuration) / dMaximumOutputStep));
dTimeGrid = zeros(double(ui32StepCount) + 1, 1);
dTimeGrid(1) = dTimeSpan(1);

for ui32StepIndex = uint32(1):ui32StepCount
    dCurrentTime = dTimeGrid(double(ui32StepIndex));
    dRemainingTime = dTimeSpan(2) - dCurrentTime;
    dSignedStep = dDirection * min( ...
        dMaximumOutputStep, abs(dRemainingTime));
    dTimeGrid(double(ui32StepIndex) + 1) = ...
        dCurrentTime + dSignedStep;
end

dTimeGrid(end) = dTimeSpan(2);
end

function ValidateDerivative_(objDerivative, objReferenceState)
% Enforce a state-sized CasADi/numerical RHS before integrator construction.
bSupportedType = isa(objDerivative, 'double') || ...
    isa(objDerivative, 'casadi.DM') || ...
    isa(objDerivative, 'casadi.SX') || ...
    isa(objDerivative, 'casadi.MX');
if ~bSupportedType || ...
        ~isequal(size(objDerivative), size(objReferenceState))
    error('PropagateAdaptiveStepAutoDiff:InvalidDerivative', ...
        'The state derivative must be a numerical or CasADi column matching the state size.');
end
if isa(objDerivative, 'double') && ~all(isfinite(objDerivative), 'all')
    error('PropagateAdaptiveStepAutoDiff:InvalidDerivative', ...
        'A numerical state derivative must contain only finite values.');
end
end
