function [dForcePerPressure_SCB, dEffectiveCr, dTransverseForcePerPressure_SCB] = ...
    EvaluateSrpResponseLut(dPosSCtoSun_SCB, strResponseLut, bIncludeTransverse) %#codegen
%% SIGNATURE
% [dForcePerPressure_SCB, dEffectiveCr, dTransverseForcePerPressure_SCB] = ...
%     EvaluateSrpResponseLut(dPosSCtoSun_SCB, strResponseLut, bIncludeTransverse)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Interpolate a spacecraft SRP response with optional transverse compensation.
% Share four bilinear weights across the scalar coefficient and nodal
% transverse force-per-pressure components. Preserve the scalar Sun-parallel response
% when enabling the correction; project the interpolated vector onto the plane
% perpendicular to the query Sun direction. Apply pressure, mass, body rotation
% and external eclipse outside this function. Validate the table once with
% ValidateSrpResponseLut before repeated evaluation.
% Apply the shared deterministic pole lookup tilt while preserving the actual
% Sun direction in scalar force and transverse projection.
% Example: [dScalar, dCr] = EvaluateSrpResponseLut([1;0;0], strResponseLut);
%          dVector = EvaluateSrpResponseLut([1;0;0], strResponseLut, true);
% Output: Body-frame scalar/vector force divided by pressure [m^2].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCtoSun_SCB                   Nonzero spacecraft-to-Sun displacement in SCB [query unit].
% strResponseLut                    Validated fixed numeric payload with uint32 active counts.
% bIncludeTransverse                Compile-time transverse correction; default false.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dForcePerPressure_SCB             Total body-frame force per unit pressure [m^2].
% dEffectiveCr                      Interpolated dimensionless Sun-parallel reflectivity [-].
% dTransverseForcePerPressure_SCB   Added transverse force per unit pressure; zero when disabled [m^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 28-09-2026  Pietro Califano, Codex gpt-6  Add optional vector response interpolation.
% 29-09-2026  Pietro Califano, Codex gpt-6  Share force interpolation with analytical derivatives.
% 29-09-2026  Pietro Califano, Codex gpt-6  Share deterministic pole lookup with derivatives.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalSrpLutKernel, ValidateSrpResponseLut (caller-side table validation).
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dPosSCtoSun_SCB (3, 1) double
    strResponseLut (1, 1) struct
    bIncludeTransverse (1, 1) logical {coder.mustBeConst} = false
end

arguments (Output)
    dForcePerPressure_SCB (3, 1) double
    dEffectiveCr (1, 1) double
    dTransverseForcePerPressure_SCB (3, 1) double
end

% Specialize the shared kernel to avoid all analytical derivative work.
[dForcePerPressure_SCB, dEffectiveCr, dTransverseForcePerPressure_SCB] = ...
    EvalSrpLutKernel(dPosSCtoSun_SCB, strResponseLut, bIncludeTransverse, false);
end
