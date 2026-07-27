function [dxAdvancedState, dAdvancedTime] = PropagateRK4Step( ...
    fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize) %#codegen
%% SIGNATURE
% [dxAdvancedState, dAdvancedTime] = PropagateRK4Step( ...
%     fcnStateDerivative, dCurrentTime, dxCurrentState, dStepSize)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Advance one state with the classical explicit fourth-order Runge-Kutta
% method. The signed step controls propagation direction; no trajectory
% storage is allocated.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnStateDerivative  (1,1) function_handle returning a state-sized column
% dCurrentTime        (1,1) double current independent-variable value
% dxCurrentState      (Nx1) double current state
% dStepSize           (1,1) double signed integration step
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxAdvancedState     (Nx1) double state at the advanced timestamp
% dAdvancedTime       (1,1) double advanced timestamp
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-07-2026  Pietro Califano, Codex     First shared SimulationGears implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    fcnStateDerivative (1,1) function_handle
    dCurrentTime (1,1) double {mustBeFinite}
    dxCurrentState (:,1) double {mustBeFinite}
    dStepSize (1,1) double {mustBeFinite}
end

arguments (Output)
    dxAdvancedState (:,1) double
    dAdvancedTime (1,1) double
end

% Evaluate the classical four stages at their matched intermediate states.
dStage1 = fcnStateDerivative(dCurrentTime, dxCurrentState);
ValidateDerivative_(dStage1, dxCurrentState);

dStage2 = fcnStateDerivative(dCurrentTime + dStepSize / 2.0, ...
    dxCurrentState + (dStepSize / 2.0) * dStage1);
ValidateDerivative_(dStage2, dxCurrentState);

dStage3 = fcnStateDerivative(dCurrentTime + dStepSize / 2.0, ...
    dxCurrentState + (dStepSize / 2.0) * dStage2);
ValidateDerivative_(dStage3, dxCurrentState);

dStage4 = fcnStateDerivative(dCurrentTime + dStepSize, ...
    dxCurrentState + dStepSize * dStage3);
ValidateDerivative_(dStage4, dxCurrentState);

% Combine the stages without allocating interval-level history.
dxAdvancedState = dxCurrentState + (dStepSize / 6.0) * ...
    (dStage1 + 2.0 * dStage2 + 2.0 * dStage3 + dStage4);
dAdvancedTime = dCurrentTime + dStepSize;

end

function ValidateDerivative_(dxDerivative, dxReferenceState)
% Enforce the public RHS dimension and finite-value contract.
if ~isa(dxDerivative, 'double') || ...
        ~isequal(size(dxDerivative), size(dxReferenceState)) || ...
        ~all(isfinite(dxDerivative), 'all')
    error('PropagateRK4Step:InvalidDerivative', ...
        'The state derivative must be a finite double column matching the state size.');
end
end
