function [objAdvancedState, dAdvancedTime] = PropagateRK4StepAutoDiff( ...
    fcnStateDerivative, dCurrentTime, objCurrentState, dStepSize, ...
    strDynParams, strModelConfigFlags)
%% SIGNATURE
% [objAdvancedState, dAdvancedTime] = PropagateRK4StepAutoDiff( ...
%     fcnStateDerivative, dCurrentTime, objCurrentState, dStepSize, ...
%     strDynParams, strModelConfigFlags)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Advance one numerical or CasADi symbolic state with the classical
% fourth-order Runge-Kutta method. Double states delegate to the canonical
% code-generation-safe numerical kernel; CasADi SX/MX states build the same
% four-stage expression without allocating trajectory storage.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% fcnStateDerivative  (1,1) function_handle using the standard four-input RHS contract
% dCurrentTime        (1,1) finite double current independent-variable value
% objCurrentState     (Nx1) finite double or CasADi SX/MX current state
% dStepSize           (1,1) finite double signed integration step
% strDynParams        (1,1) struct runtime dynamics payload forwarded to RHS
% strModelConfigFlags (1,1) struct model selection forwarded to RHS
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objAdvancedState    (Nx1) double or CasADi SX/MX state at the advanced time
% dAdvancedTime       (1,1) double advanced timestamp
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-07-2026  Pietro Califano, Codex     Restore numerical and symbolic RK4 stepping.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% PropagateRK4Step
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    fcnStateDerivative (1,1) function_handle
    dCurrentTime (1,1) double {mustBeFinite}
    objCurrentState (:,1)
    dStepSize (1,1) double {mustBeFinite}
    strDynParams (1,1) struct
    strModelConfigFlags (1,1) struct
end

arguments (Output)
    objAdvancedState (:,1)
    dAdvancedTime (1,1) double
end

if isa(objCurrentState, 'double')
    % Reuse the canonical numerical implementation so the AutoDiff boundary
    % cannot drift from generated-code RK4 behavior.
    [objAdvancedState, dAdvancedTime] = PropagateRK4Step( ...
        fcnStateDerivative, dCurrentTime, objCurrentState, dStepSize, ...
        strDynParams, strModelConfigFlags);
    return;
end

ValidateSymbolicState_(objCurrentState);

% Evaluate the classical four stages directly on the symbolic state graph.
objStage1 = fcnStateDerivative( ...
    dCurrentTime, objCurrentState, strDynParams, strModelConfigFlags);
ValidateDerivative_(objStage1, objCurrentState);

objStage2 = fcnStateDerivative(dCurrentTime + dStepSize / 2.0, ...
    objCurrentState + (dStepSize / 2.0) * objStage1, ...
    strDynParams, strModelConfigFlags);
ValidateDerivative_(objStage2, objCurrentState);

objStage3 = fcnStateDerivative(dCurrentTime + dStepSize / 2.0, ...
    objCurrentState + (dStepSize / 2.0) * objStage2, ...
    strDynParams, strModelConfigFlags);
ValidateDerivative_(objStage3, objCurrentState);

objStage4 = fcnStateDerivative(dCurrentTime + dStepSize, ...
    objCurrentState + dStepSize * objStage3, ...
    strDynParams, strModelConfigFlags);
ValidateDerivative_(objStage4, objCurrentState);

% Combine stages without forcing symbolic values through a double buffer.
objAdvancedState = objCurrentState + (dStepSize / 6.0) * ...
    (objStage1 + 2.0 * objStage2 + 2.0 * objStage3 + objStage4);
dAdvancedTime = dCurrentTime + dStepSize;

end

function ValidateSymbolicState_(objCurrentState)
% Restrict the symbolic branch to differentiable CasADi graph types.
if ~(isa(objCurrentState, 'casadi.SX') || ...
        isa(objCurrentState, 'casadi.MX'))
    error('PropagateRK4StepAutoDiff:InvalidStateType', ...
        'The state must be a finite double column or a CasADi SX/MX column.');
end
if isempty(objCurrentState)
    error('PropagateRK4StepAutoDiff:EmptyState', ...
        'The current state must contain at least one element.');
end
end

function ValidateDerivative_(objDerivative, objReferenceState)
% Enforce state-sized numerical or symbolic RHS output at every stage.
bSupportedType = isa(objDerivative, 'double') || ...
    isa(objDerivative, 'casadi.DM') || ...
    isa(objDerivative, 'casadi.SX') || ...
    isa(objDerivative, 'casadi.MX');
if ~bSupportedType || ...
        ~isequal(size(objDerivative), size(objReferenceState))
    error('PropagateRK4StepAutoDiff:InvalidDerivative', ...
        'The state derivative must be a numerical or CasADi column matching the state size.');
end
if isa(objDerivative, 'double') && ~all(isfinite(objDerivative), 'all')
    error('PropagateRK4StepAutoDiff:InvalidDerivative', ...
        'A numerical state derivative must contain only finite values.');
end
end
