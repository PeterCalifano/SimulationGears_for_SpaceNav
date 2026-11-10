function dJacSHE_TB = EvalJac_ExtSphHarmExpInTargetFrame(dRSC_TB, ...
    ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
%% SIGNATURE
% dJacSHE_TB = EvalJac_ExtSphHarmExpInTargetFrame(dRSC_TB, ui32MaxDegree, ...
%     dCSlmCoeffCols, dGravParam, dBodyRadiusRef)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate the analytical Cartesian Hessian of the exterior SH perturbing potential.
% Request the third output of the existing target-frame evaluator. Keep this
% compatibility entry point for dynamics and generated Jacobian consumers.
% Cartesian recurrences remain regular on the polar axis; no finite differences,
% symmetry projection or trace correction enter the derivative. Coefficient row
% two starts degree two, preserving the existing ExtSHE column-pair convention.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRSC_TB          Target-frame position [LU].
% ui32MaxDegree    Active maximum harmonic degree.
% dCSlmCoeffCols   Unnormalized cosine/sine pairs in ExtSHE storage.
% dGravParam       Gravitational parameter [LU^3/s^2].
% dBodyRadiusRef   Harmonic reference radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacSHE_TB       Perturbing acceleration position Jacobian [s^-2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5.5  Restore canonical SH Jacobian function for RHS Jacobian use.
% 28-05-2026    Pietro Califano, Codex 5.5  Improve implementation and documentation.
% 07-10-2026    Pietro Califano, Codex      Replace numerical gravity partials with analytical recurrences.
% 10-10-2026    Pietro Califano            Preserve historical entries and delegate to the shared evaluator.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalExtSphHarmExpInTargetFrame.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dRSC_TB         (3,1) double {mustBeReal, mustBeFinite}
    ui32MaxDegree   (1,1) uint32
    dCSlmCoeffCols  (:,2) double {mustBeReal, mustBeFinite}
    dGravParam      (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
    dBodyRadiusRef  (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
end
arguments (Output)
    dJacSHE_TB      (3,3) double
end

% Keep this entry point's established error identifiers ahead of the shared evaluator.
assert(norm(dRSC_TB) > 0, 'EvalJac_ExtSphHarmExpInTargetFrame:ZeroPosition', ...
    'Position vector must be nonzero.');

dRequiredRows = (double(ui32MaxDegree) + 1) * (double(ui32MaxDegree) + 2) / 2 - 2;
assert(ui32MaxDegree < uint32(2) || size(dCSlmCoeffCols, 1) >= dRequiredRows, ...
    'EvalJac_ExtSphHarmExpInTargetFrame:InsufficientCoefficients', ...
    'Coefficient storage must cover the requested harmonic degree.');

[~, ~, dJacSHE_TB] = EvalExtSphHarmExpInTargetFrame(dRSC_TB, ui32MaxDegree, dCSlmCoeffCols, ...
    dGravParam, dBodyRadiusRef);
end
