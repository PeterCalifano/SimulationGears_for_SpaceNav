function [dJacForcePerPressWrtSunPos_SCB, dJacCrWrtSunPos_SCB, ...
    dJacTransverseWrtSunPos_SCB, dForcePerPressure_SCB, dEffectiveCr, ...
    dTransverseForcePerPressure_SCB, bDerivativeRegular] = ...
    EvalJac_SrpResponseLut(dPosSCtoSun_SCB, strResponseLut, bIncludeTransverse) %#codegen
%% SIGNATURE
% [dJacForcePerPressWrtSunPos_SCB, dJacCrWrtSunPos_SCB, dJacTransverseWrtSunPos_SCB, ...
%     dForcePerPressure_SCB, dEffectiveCr, dTransverseForcePerPressure_SCB, ...
%     bDerivativeRegular] = EvalJac_SrpResponseLut(dPosSCtoSun_SCB, ...
%     strResponseLut, bIncludeTransverse)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate analytical SRP response derivatives and force in one interpolation.
% Differentiate the supplied non-unit query, not only its spherical angles.
% Keep the scalar projection independent of the optional transverse term.
% Use the selected one-sided cell partial at knots/seam. Near either pole,
% differentiate the deterministic tiny lookup tilt used by force evaluation
% and return false derivative regularity to identify the adjusted query.
% Example: [dJac, ~, ~, dForce] = EvalJac_SrpResponseLut([1;0.2;0.3], strResponseLut, true);
% Output: A 3-by-3 body-response partial and force per pressure [m^2].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCtoSun_SCB                   Nonzero spacecraft-to-Sun displacement in SCB [query unit].
% strResponseLut                    Immutable payload validated during preparation.
% bIncludeTransverse                Compile-time transverse selection; default false.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacForcePerPressWrtSunPos_SCB    Total response/query partial [m^2/query unit].
% dJacCrWrtSunPos_SCB               Parallel coefficient/query gradient [1/query unit].
% dJacTransverseWrtSunPos_SCB       Transverse response/query partial [m^2/query unit].
% dForcePerPressure_SCB             Total body-frame force/pressure [m^2].
% dEffectiveCr                      Dimensionless parallel coefficient.
% dTransverseForcePerPressure_SCB   Optional transverse force/pressure [m^2].
% bDerivativeRegular                True away from knots/seam/poles; see DESCRIPTION.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Add analytical scalar/vector LUT partials.
% 29-09-2026  Pietro Califano, Codex gpt-6  Differentiate deterministic pole lookup adjustment.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalSrpLutKernel, ValidateSrpResponseLut (host preparation).
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dPosSCtoSun_SCB (3, 1) double
    strResponseLut (1, 1) struct
    bIncludeTransverse (1, 1) logical {coder.mustBeConst} = false
end

arguments (Output)
    dJacForcePerPressWrtSunPos_SCB (3, 3) double
    dJacCrWrtSunPos_SCB (1, 3) double
    dJacTransverseWrtSunPos_SCB (3, 3) double
    dForcePerPressure_SCB (3, 1) double
    dEffectiveCr (1, 1) double
    dTransverseForcePerPressure_SCB (3, 1) double
    bDerivativeRegular (1, 1) logical
end

% Reuse one interpolation for the force and every requested analytical partial.
[dForcePerPressure_SCB, dEffectiveCr, dTransverseForcePerPressure_SCB, ...
    dJacForcePerPressWrtSunPos_SCB, dJacCrWrtSunPos_SCB, dJacTransverseWrtSunPos_SCB, ...
    bDerivativeRegular] = EvalSrpLutKernel(dPosSCtoSun_SCB, ...
                                       strResponseLut, bIncludeTransverse, true);
end
