function [dxAdvancedState, dAdvancedTime] = PropagateRK2HeunStep( ...
    fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize, ...
    strDynParams, strModelConfigFlags) %#codegen
%% SIGNATURE
% [dxAdvancedState, dAdvancedTime] = PropagateRK2HeunStep( ...
%     fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize, ...
%     strDynParams, strModelConfigFlags)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Advance one state with Heun's explicit second-order Runge-Kutta method.
% The signed step controls propagation direction; no trajectory storage is
% allocated.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnStateDerivative  (1,1) function_handle returning a state-sized column
% dCurrentTime        (1,1) double current independent-variable value
% dxCurrentState      (Nx1) double current state
% dStepSize           (1,1) double signed integration step
% strDynParams        (1,1) struct runtime dynamics payload forwarded to RHS
% strModelConfigFlags (1,1) struct compile-time model selection forwarded to RHS
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxAdvancedState     (Nx1) double state at the advanced timestamp
% dAdvancedTime       (1,1) double advanced timestamp
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-07-2026  Pietro Califano, Codex     First shared SimulationGears implementation.
% 27-07-2026  Pietro Califano, Codex     Add explicit standard RHS inputs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    fcnStateDerivative (1,1) function_handle
    dCurrentTime (1,1) double {mustBeFinite}
    dxCurrentState (:,1) double {mustBeFinite}
    dStepSize (1,1) double {mustBeFinite}
    strDynParams (1,1) struct
    strModelConfigFlags (1,1) struct {coder.mustBeConst}
end

arguments (Output)
    dxAdvancedState (:,1) double
    dAdvancedTime (1,1) double
end

% Evaluate the tangent at the interval start and at the Euler predictor.
dStage1 = fcnStateDerivative( ...
    dCurrentTime, dxCurrentState, strDynParams, strModelConfigFlags);
ValidateDerivative_(dStage1, dxCurrentState);

dxPredictedState = dxCurrentState + dStepSize * dStage1;
dStage2 = fcnStateDerivative(dCurrentTime + dStepSize, dxPredictedState, ...
    strDynParams, strModelConfigFlags);
ValidateDerivative_(dStage2, dxCurrentState);

% Average the endpoint tangents to obtain the second-order correction.
dxAdvancedState = dxCurrentState + (dStepSize / 2.0) * ...
    (dStage1 + dStage2);
dAdvancedTime = dCurrentTime + dStepSize;

end

function ValidateDerivative_(dxDerivative, dxReferenceState)
% Enforce the public RHS dimension and finite-value contract.
if ~isa(dxDerivative, 'double') || ...
        ~isequal(size(dxDerivative), size(dxReferenceState)) || ...
        ~all(isfinite(dxDerivative), 'all')
    error('PropagateRK2HeunStep:InvalidDerivative', ...
        'The state derivative must be a finite double column matching the state size.');
end
end
