function dFinalState = CodegenFixedStepSchemeProbe(enumFixedStepScheme) %#codegen
%% SIGNATURE
% dFinalState = CodegenFixedStepSchemeProbe(enumFixedStepScheme)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Exercise the fixed-step propagation entry point with a code-generation
% scheme input. This test fixture keeps every other propagation input
% compile-time fixed so the regression isolates scheme specialization.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumFixedStepScheme  (1,1) EnumFixedStepScheme selected integration scheme
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dFinalState          (1,1) double final state after one fixed interval
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-07-2026  Pietro Califano, Codex     First code-generation probe.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EnumFixedStepScheme, PropagateFixedStep
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    enumFixedStepScheme (1,1) EnumFixedStepScheme
end

arguments (Output)
    dFinalState (1,1) double
end

dxStateHistory = PropagateFixedStep(@GrowthRhs_, ...
    [0.0, 1.0], 1.0, 0.25, enumFixedStepScheme);
dFinalState = dxStateHistory(end);
end

function dxStateDerivative = GrowthRhs_(dTime, dxState)
% Return scalar exponential-growth dynamics for generated-code validation.
dxStateDerivative = dxState + 0.0 * dTime;
end
