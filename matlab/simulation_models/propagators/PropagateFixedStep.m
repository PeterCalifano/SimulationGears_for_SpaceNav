function [dxStateHistory, dTimeGrid, strStatistics] = PropagateFixedStep( ...
    fcnStateDerivative, dTimeSpan, dxInitialState, dMaximumStep, ...
    strDynParams, strModelConfigFlags, enumFixedStepScheme) %#codegen
%% SIGNATURE
% [dxStateHistory, dTimeGrid, strStatistics] = PropagateFixedStep( ...
%     fcnStateDerivative, dTimeSpan, dxInitialState, dMaximumStep, ...
%     strDynParams, strModelConfigFlags, enumFixedStepScheme)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Propagate a state over one interval with the selected fixed-step numerical
% integrator. The final step is shortened when necessary so the returned
% column time grid contains the requested endpoint exactly. State samples are
% stored one per row.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnStateDerivative  (1,1) function_handle returning a state-sized column
% dTimeSpan           (1,2) double start and end timestamps
% dxInitialState      (Nx1) double initial state
% dMaximumStep        (1,1) double positive maximum step magnitude
% strDynParams        (1,1) struct runtime dynamics payload forwarded to RHS
% strModelConfigFlags (1,1) struct model selection forwarded to RHS; must be
%                     compile-time constant for generated code
% enumFixedStepScheme (1,1) EnumFixedStepScheme integration scheme; must be
%                     compile-time constant for generated code
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxStateHistory      (MxN) double state history with one state per row
% dTimeGrid           (Mx1) double timestamps corresponding to history rows
% strStatistics       (1,1) struct accepted/rejected step and evaluation data
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-07-2026  Pietro Califano, Codex     First shared fixed-step propagator.
% 27-07-2026  Pietro Califano, Codex     Require codegen scheme specialization.
% 27-07-2026  Pietro Califano, Codex     Add explicit standard RHS inputs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EnumFixedStepScheme, PropagateRK2HeunStep, PropagateRK4Step,
% PropagateRK8Step
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    fcnStateDerivative (1,1) function_handle
    dTimeSpan (1,2) double {mustBeFinite}
    dxInitialState (:,1) double {mustBeFinite}
    dMaximumStep (1,1) double {mustBeFinite, mustBePositive}
    strDynParams (1,1) struct
    strModelConfigFlags (1,1) struct {coder.mustBeConst}
    enumFixedStepScheme (1,1) EnumFixedStepScheme {coder.mustBeConst}
end

arguments (Output)
    dxStateHistory (:,:) double
    dTimeGrid (:,1) double
    strStatistics (1,1) struct
end

if isempty(dxInitialState)
    error('PropagateFixedStep:EmptyInitialState', ...
        'The initial state must contain at least one element.');
end

% Resolve the evaluation count once
switch enumFixedStepScheme
    case EnumFixedStepScheme.RK2Heun
        ui32EvaluationsPerStep = uint32(2);
    case EnumFixedStepScheme.RK4
        ui32EvaluationsPerStep = uint32(4);
    case EnumFixedStepScheme.RK8
        ui32EvaluationsPerStep = uint32(13);
    otherwise
        error('PropagateFixedStep:UnsupportedScheme', ...
            'The fixed-step integration scheme is unsupported.');
end

dDuration = dTimeSpan(2) - dTimeSpan(1);
ui32StepCount = uint32(ceil(abs(dDuration) / dMaximumStep));
ui32StateSize = uint32(numel(dxInitialState));

% Allocate the complete fixed-step trajectory from the known interval length.
dxStateHistory = zeros(double(ui32StepCount) + 1, double(ui32StateSize));
dTimeGrid = zeros(double(ui32StepCount) + 1, 1);
dxStateHistory(1, :) = dxInitialState.';
dTimeGrid(1) = dTimeSpan(1);

% Advance with a signed maximum step and shorten only the final interval.
dDirection = sign(dDuration);
for ui32StepIndex = uint32(1):ui32StepCount

    dCurrentTime = dTimeGrid(double(ui32StepIndex));
    dRemainingTime = dTimeSpan(2) - dCurrentTime;
    dStepSize = dDirection * min(dMaximumStep, abs(dRemainingTime));
    dxCurrentState = dxStateHistory(double(ui32StepIndex), :).';
    
    switch enumFixedStepScheme
        case EnumFixedStepScheme.RK2Heun
            [dxAdvancedState, dAdvancedTime] = PropagateRK2HeunStep( ...
                fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize, ...
                strDynParams, strModelConfigFlags);
        case EnumFixedStepScheme.RK4
            [dxAdvancedState, dAdvancedTime] = PropagateRK4Step( ...
                fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize, ...
                strDynParams, strModelConfigFlags);
        case EnumFixedStepScheme.RK8
            [dxAdvancedState, dAdvancedTime] = PropagateRK8Step( ...
                fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize, ...
                strDynParams, strModelConfigFlags);
        otherwise
            error('PropagateFixedStep:UnsupportedScheme', ...
                'The fixed-step integration scheme is unsupported.');
    end

    dxStateHistory(double(ui32StepIndex) + 1, :) = dxAdvancedState.';
    dTimeGrid(double(ui32StepIndex) + 1) = dAdvancedTime;
end

if ui32StepCount > 0
    dTimeGrid(end) = dTimeSpan(2);
    dAcceptedStepSizes = abs(diff(dTimeGrid));
    dMinimumAcceptedStep = min(dAcceptedStepSizes);
    dMaximumAcceptedStep = max(dAcceptedStepSizes);
    dMeanAcceptedStep = mean(dAcceptedStepSizes);
else
    dMinimumAcceptedStep = 0.0;
    dMaximumAcceptedStep = 0.0;
    dMeanAcceptedStep = 0.0;
end

% Use one statistics layout across fixed and adaptive propagators.
strStatistics = struct( ...
    'ui32AcceptedSteps', ui32StepCount, ...
    'ui32RejectedSteps', uint32(0), ...
    'ui32FunctionEvaluations', ui32EvaluationsPerStep * ui32StepCount, ...
    'dMinimumAcceptedStep', dMinimumAcceptedStep, ...
    'dMaximumAcceptedStep', dMaximumAcceptedStep, ...
    'dMeanAcceptedStep', dMeanAcceptedStep);

end
