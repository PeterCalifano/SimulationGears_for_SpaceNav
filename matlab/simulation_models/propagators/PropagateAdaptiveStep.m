function [dxStateHistory, dTimeGrid, strStatistics] = PropagateAdaptiveStep( ...
    fcnStateDerivative, dTimeSpan, dxInitialState, dInitialStep, settings) %#codegen
%% SIGNATURE
% [dxStateHistory, dTimeGrid, strStatistics] = PropagateAdaptiveStep( ...
%     fcnStateDerivative, dTimeSpan, dxInitialState, dInitialStep, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Propagate a state with the embedded Runge-Kutta-Fehlberg 4(5) pair and
% adaptive step control. Accepted fifth-order states are returned one per row;
% rejected attempts are counted but never enter the trajectory. Forward,
% backward, shortened-final-step, and zero-duration intervals are supported.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnStateDerivative              (1,1) function_handle returning a state-sized column
% dTimeSpan                       (1,2) double start and end timestamps
% dxInitialState                  (Nx1) double initial state
% dInitialStep                    (1,1) double positive initial step magnitude
% settings.dRelativeTolerance     (1,1) double positive relative tolerance
% settings.dAbsoluteTolerance     (1,1) double positive absolute tolerance
% settings.dMinimumStep           (1,1) double nonnegative minimum step magnitude
% settings.dMaximumStep           (1,1) double positive maximum step magnitude
% settings.ui32MaximumAttempts    (1,1) uint32 maximum accepted plus rejected attempts
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxStateHistory      (MxN) double accepted state history with one state per row
% dTimeGrid           (Mx1) double timestamps corresponding to history rows
% strStatistics       (1,1) struct accepted/rejected step and evaluation data
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-07-2026  Pietro Califano, Codex     Repair and migrate the adaptive interval propagator.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    fcnStateDerivative (1,1) function_handle
    dTimeSpan (1,2) double {mustBeFinite}
    dxInitialState (:,1) double {mustBeFinite}
    dInitialStep (1,1) double {mustBeFinite, mustBePositive}
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
    dxStateHistory (:,:) double
    dTimeGrid (:,1) double
    strStatistics (1,1) struct
end

if isempty(dxInitialState)
    error('PropagateAdaptiveStep:EmptyInitialState', ...
        'The initial state must contain at least one element.');
end
if settings.dMinimumStep > settings.dMaximumStep
    error('PropagateAdaptiveStep:InvalidStepBounds', ...
        'The minimum step cannot exceed the maximum step.');
end

dDuration = dTimeSpan(2) - dTimeSpan(1);
if dDuration == 0.0
    dxStateHistory = dxInitialState.';
    dTimeGrid = dTimeSpan(1);
    strStatistics = BuildStatistics_(uint32(0), uint32(0), ...
        zeros(0, 1));
    return;
end

dDirection = sign(dDuration);
dCurrentStep = min(dInitialStep, settings.dMaximumStep);
dScaleTime = max([1.0, abs(dTimeSpan)]);
dMachineMinimumStep = 16.0 * eps(dScaleTime);
dEffectiveMinimumStep = max(settings.dMinimumStep, dMachineMinimumStep);

% Start with a bounded estimate and grow geometrically only when adaptation
% accepts more states than the initial-step estimate predicts.
dEstimatedSamples = ceil(abs(dDuration) / dCurrentStep) + 1;
dCapacity = max(16.0, min(1024.0, dEstimatedSamples));
dxStateBuffer = zeros(dCapacity, numel(dxInitialState));
dTimeBuffer = zeros(dCapacity, 1);
dxStateBuffer(1, :) = dxInitialState.';
dTimeBuffer(1) = dTimeSpan(1);

dxCurrentState = dxInitialState;
dCurrentTime = dTimeSpan(1);
ui32AcceptedSteps = uint32(0);
ui32RejectedSteps = uint32(0);
dAcceptedStepBuffer = zeros(dCapacity - 1, 1);

while dDirection * (dTimeSpan(2) - dCurrentTime) > 0.0
    
    ui32AttemptCount = ui32AcceptedSteps + ui32RejectedSteps;
    if ui32AttemptCount >= settings.ui32MaximumAttempts
        error('PropagateAdaptiveStep:MaximumAttemptsExceeded', ...
            'The adaptive propagator exceeded the configured attempt limit.');
    end

    dRemainingTime = abs(dTimeSpan(2) - dCurrentTime);
    dAttemptStep = min([dCurrentStep, settings.dMaximumStep, dRemainingTime]);
    if dAttemptStep < dEffectiveMinimumStep && ...
            dRemainingTime > dEffectiveMinimumStep
        error('PropagateAdaptiveStep:MinimumStepExceeded', ...
            'The requested tolerance requires a step below the allowed minimum.');
    end

    dSignedStep = dDirection * dAttemptStep;
    dStageValues = EvalFehlbergStages_(fcnStateDerivative, ...
        dCurrentTime, dxCurrentState, dSignedStep);

    % Evaluate both embedded solutions and normalize their difference against
    % a component-wise mixed absolute/relative tolerance.
    dxFourthOrderState = dxCurrentState + dSignedStep * ...
        (25.0 / 216.0 * dStageValues(:, 1) + ...
         1408.0 / 2565.0 * dStageValues(:, 3) + ...
         2197.0 / 4104.0 * dStageValues(:, 4) - ...
         1.0 / 5.0 * dStageValues(:, 5));
    dxFifthOrderState = dxCurrentState + dSignedStep * ...
        (16.0 / 135.0 * dStageValues(:, 1) + ...
         6656.0 / 12825.0 * dStageValues(:, 3) + ...
         28561.0 / 56430.0 * dStageValues(:, 4) - ...
         9.0 / 50.0 * dStageValues(:, 5) + ...
         2.0 / 55.0 * dStageValues(:, 6));

    dErrorScale = settings.dAbsoluteTolerance + ...
        settings.dRelativeTolerance * ...
        max(abs(dxCurrentState), abs(dxFifthOrderState));
    dNormalizedError = max(abs(dxFifthOrderState - ...
        dxFourthOrderState) ./ dErrorScale);

    if dNormalizedError == 0.0
        dStepFactor = 5.0;
    else
        dStepFactor = 0.9 * dNormalizedError^(-1.0 / 5.0);
        dStepFactor = min(5.0, max(0.2, dStepFactor));
    end

    if dNormalizedError <= 1.0
        ui32AcceptedSteps = ui32AcceptedSteps + uint32(1);
        dCurrentTime = dCurrentTime + dSignedStep;
        if dAttemptStep == dRemainingTime
            dCurrentTime = dTimeSpan(2);
        end
        dxCurrentState = dxFifthOrderState;

        dRequiredSampleCount = double(ui32AcceptedSteps) + 1.0;
        if dRequiredSampleCount > dCapacity
            [dxStateBuffer, dTimeBuffer, dAcceptedStepBuffer, dCapacity] = ...
                GrowBuffers_(dxStateBuffer, dTimeBuffer, ...
                dAcceptedStepBuffer, dCapacity);
        end
        dxStateBuffer(dRequiredSampleCount, :) = dxCurrentState.';
        dTimeBuffer(dRequiredSampleCount) = dCurrentTime;
        dAcceptedStepBuffer(double(ui32AcceptedSteps)) = dAttemptStep;
    else
        ui32RejectedSteps = ui32RejectedSteps + uint32(1);
        dStepFactor = min(1.0, dStepFactor);
    end

    dProposedStep = dAttemptStep * dStepFactor;
    if dNormalizedError > 1.0 && ...
            dProposedStep < dEffectiveMinimumStep
        error('PropagateAdaptiveStep:MinimumStepExceeded', ...
            'The requested tolerance requires a step below the allowed minimum.');
    end
    dCurrentStep = min(settings.dMaximumStep, ...
        max(dEffectiveMinimumStep, dProposedStep));
end

dSampleCount = double(ui32AcceptedSteps) + 1.0;
dxStateHistory = dxStateBuffer(1:dSampleCount, :);
dTimeGrid = dTimeBuffer(1:dSampleCount);
dTimeGrid(end) = dTimeSpan(2);
dAcceptedStepSizes = dAcceptedStepBuffer(1:double(ui32AcceptedSteps));
strStatistics = BuildStatistics_(ui32AcceptedSteps, ...
    ui32RejectedSteps, dAcceptedStepSizes);

end

function dStageValues = EvalFehlbergStages_( ...
    fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize)
% Evaluate the six stages of the Fehlberg embedded 4(5) pair.
dStageValues = zeros(numel(dxCurrentState), 6);

dStageValues(:, 1) = EvaluateDerivative_(fcnStateDerivative, ...
    dCurrentTime, dxCurrentState, dxCurrentState);

dStageValues(:, 2) = EvaluateDerivative_(fcnStateDerivative, ...
    dCurrentTime + dStepSize / 4.0, ...
    dxCurrentState + dStepSize * dStageValues(:, 1) / 4.0, ...
    dxCurrentState);

dStageValues(:, 3) = EvaluateDerivative_(fcnStateDerivative, ...
    dCurrentTime + 3.0 * dStepSize / 8.0, ...
    dxCurrentState + dStepSize * ...
    (3.0 / 32.0 * dStageValues(:, 1) + ...
     9.0 / 32.0 * dStageValues(:, 2)), dxCurrentState);

dStageValues(:, 4) = EvaluateDerivative_(fcnStateDerivative, ...
    dCurrentTime + 12.0 * dStepSize / 13.0, ...
    dxCurrentState + dStepSize * ...
    (1932.0 / 2197.0 * dStageValues(:, 1) - ...
     7200.0 / 2197.0 * dStageValues(:, 2) + ...
     7296.0 / 2197.0 * dStageValues(:, 3)), dxCurrentState);

dStageValues(:, 5) = EvaluateDerivative_(fcnStateDerivative, ...
    dCurrentTime + dStepSize, ...
    dxCurrentState + dStepSize * ...
    (439.0 / 216.0 * dStageValues(:, 1) - ...
     8.0 * dStageValues(:, 2) + ...
     3680.0 / 513.0 * dStageValues(:, 3) - ...
     845.0 / 4104.0 * dStageValues(:, 4)), dxCurrentState);

dStageValues(:, 6) = EvaluateDerivative_(fcnStateDerivative, ...
    dCurrentTime + dStepSize / 2.0, ...
    dxCurrentState + dStepSize * ...
    (-8.0 / 27.0 * dStageValues(:, 1) + ...
     2.0 * dStageValues(:, 2) - ...
     3544.0 / 2565.0 * dStageValues(:, 3) + ...
     1859.0 / 4104.0 * dStageValues(:, 4) - ...
     11.0 / 40.0 * dStageValues(:, 5)), dxCurrentState);
end

function dxDerivative = EvaluateDerivative_( ...
    fcnStateDerivative, dEvaluationTime, dxEvaluationState, dxReferenceState)
% Validate a raw RHS result before stage-buffer assignment can expand it.
dxDerivative = fcnStateDerivative(dEvaluationTime, dxEvaluationState);
ValidateDerivative_(dxDerivative, dxReferenceState);
end

function ValidateDerivative_(dxDerivative, dxReferenceState)
% Enforce the public RHS dimension and finite-value contract.
if ~isa(dxDerivative, 'double') || ...
        ~isequal(size(dxDerivative), size(dxReferenceState)) || ...
        ~all(isfinite(dxDerivative), 'all')
    error('PropagateAdaptiveStep:InvalidDerivative', ...
        'The state derivative must be a finite double column matching the state size.');
end
end

function [dxStateBuffer, dTimeBuffer, dStepBuffer, dCapacity] = ...
    GrowBuffers_(dxStateBuffer, dTimeBuffer, dStepBuffer, dCapacity)
% Double adaptive trajectory storage without incremental array growth.
dNewCapacity = 2.0 * dCapacity;
dxNewStateBuffer = zeros(dNewCapacity, size(dxStateBuffer, 2));
dNewTimeBuffer = zeros(dNewCapacity, 1);
dNewStepBuffer = zeros(dNewCapacity - 1.0, 1);

dxNewStateBuffer(1:dCapacity, :) = dxStateBuffer;
dNewTimeBuffer(1:dCapacity) = dTimeBuffer;
dNewStepBuffer(1:dCapacity - 1.0) = dStepBuffer;

dxStateBuffer = dxNewStateBuffer;
dTimeBuffer = dNewTimeBuffer;
dStepBuffer = dNewStepBuffer;
dCapacity = dNewCapacity;
end

function strStatistics = BuildStatistics_( ...
    ui32AcceptedSteps, ui32RejectedSteps, dAcceptedStepSizes)
% Build the common propagation-statistics layout.
if isempty(dAcceptedStepSizes)
    dMinimumAcceptedStep = 0.0;
    dMaximumAcceptedStep = 0.0;
    dMeanAcceptedStep = 0.0;
else
    dMinimumAcceptedStep = min(dAcceptedStepSizes);
    dMaximumAcceptedStep = max(dAcceptedStepSizes);
    dMeanAcceptedStep = mean(dAcceptedStepSizes);
end

strStatistics = struct( ...
    'ui32AcceptedSteps', ui32AcceptedSteps, ...
    'ui32RejectedSteps', ui32RejectedSteps, ...
    'ui32FunctionEvaluations', uint32(6) * ...
    (ui32AcceptedSteps + ui32RejectedSteps), ...
    'dMinimumAcceptedStep', dMinimumAcceptedStep, ...
    'dMaximumAcceptedStep', dMaximumAcceptedStep, ...
    'dMeanAcceptedStep', dMeanAcceptedStep);
end
