function [dxStateHistory, dTimeGrid] = ...
    CodegenInertialFixedStepProbe( ...
    dTimeSpan, dxInitialState, dMaximumStep, strDynParams, ...
    strModelConfigFlags, enumFixedStepScheme) %#codegen
%% SIGNATURE
% [dxStateHistory, dTimeGrid] = CodegenInertialFixedStepProbe( ...
%     dTimeSpan, dxInitialState, dMaximumStep, strDynParams, ...
%     strModelConfigFlags, enumFixedStepScheme)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Bind the max-fidelity inertial RHS to the shared fixed-step provider for
% generated-code testing. This fixture contains no integration algorithm; it
% exists because MATLAB Coder R2024b does not accept a function handle inside
% a coder.Constant entry-point argument.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTimeSpan           (1,2) double propagation start and end timestamps
% dxInitialState      (Nx1) double initial state
% dMaximumStep        (1,1) double positive maximum step magnitude
% strDynParams        (1,1) struct runtime max-fidelity dynamics payload
% strModelConfigFlags (1,1) struct compile-time force-model selection
% enumFixedStepScheme (1,1) EnumFixedStepScheme compile-time scheme selection
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxStateHistory      (MxN) double state history with one state per row
% dTimeGrid           (Mx1) double timestamps corresponding to history rows
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-07-2026  Pietro Califano, Codex     First generated-code provider probe.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% PropagateFixedStep, evalRHS_InertialDynMaxFidelity
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dTimeSpan (1,2) double
    dxInitialState (:,1) double
    dMaximumStep (1,1) double
    strDynParams (1,1) struct
    strModelConfigFlags (1,1) struct {coder.mustBeConst}
    enumFixedStepScheme (1,1) EnumFixedStepScheme {coder.mustBeConst}
end

arguments (Output)
    dxStateHistory (:,:) double
    dTimeGrid (:,1) double
end

[dxStateHistory, dTimeGrid] = PropagateFixedStep( ...
    @evalRHS_InertialDynMaxFidelity, dTimeSpan, dxInitialState, ...
    dMaximumStep, strDynParams, strModelConfigFlags, enumFixedStepScheme);

end
