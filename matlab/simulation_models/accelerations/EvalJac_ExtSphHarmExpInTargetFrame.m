function dJacSHE_TB = EvalJac_ExtSphHarmExpInTargetFrame(dRSC_TB, ...
    ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
arguments
    dRSC_TB             (3,1) double {mustBeReal, mustBeFinite}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeReal, mustBeFinite}
    dGravParam          (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
end
%% PROTOTYPE
% dJacSHE_TB = EvalJac_ExtSphHarmExpInTargetFrame(dRSC_TB, ui32MaxDegree, ...
%     dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Canonical target-frame Jacobian evaluator for the exterior spherical-harmonics acceleration perturbation.
% The Jacobian is evaluated by central finite differences of EvalExtSphHarmExpInTargetFrame and projected
% onto the symmetric trace-free subspace expected for an exterior conservative gravity perturbation.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dRSC_TB            (3,1) double   Spacecraft position in target-body-fixed frame.
% ui32MaxDegree      (1,1) uint32   Maximum spherical-harmonics degree.
% dCSlmCoeffCols     (:,2) double   Unnormalized [Clm, Slm] coefficient column pairs.
% dGravParam         (1,1) double   Body gravitational parameter.
% dBodyRadiusRef     (1,1) double   Body reference radius.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dJacSHE_TB         (3,3) double   d(a_SH)/d(r) in target-body-fixed coordinates.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-04-2026    Pietro Califano, Codex 5.5      Restore canonical SH Jacobian function for RHS Jacobian use.
% 28-05-2026    Pietro Califano, Codex 5.5      Minor improvements and documentation
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalExtSphHarmExpInTargetFrame()
% -------------------------------------------------------------------------------------------------------------

%% Function code
dPosNorm = norm(dRSC_TB);

% Check for valid input conditions
if dPosNorm <= 0.0
    error('EvalJac_ExtSphHarmExpInTargetFrame:ZeroPosition', ...
        'Position vector must be non-zero.');
end

if ui32MaxDegree >= uint32(2)
    ui32RequiredRows = ((ui32MaxDegree + uint32(1)) * (ui32MaxDegree + uint32(2))) ...
        / uint32(2) - uint32(2);

    if size(dCSlmCoeffCols, 1) < double(ui32RequiredRows)
        error('EvalJac_ExtSphHarmExpInTargetFrame:InsufficientCoefficients', ...
            ['dCSlmCoeffCols has %d rows, but degree %d requires at least %d ' ...
            'rows in ExtSHE column-pair format.'], ...
            size(dCSlmCoeffCols, 1), double(ui32MaxDegree), double(ui32RequiredRows));
    end
end

% Determine FD perturbation step
dStep = 1.0e-6 * max(dPosNorm, dBodyRadiusRef);

% Central finite difference approximation of the Jacobian
dJacSHE_TB = zeros(3, 3);

for idxAxis = 1:3

    dPerturb = zeros(3, 1);
    dPerturb(idxAxis) = dStep;

    % Evaluate the acceleration at the perturbed positions
    [~, dAccPlus] = EvalExtSphHarmExpInTargetFrame( ...
        dRSC_TB + dPerturb, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef);
    [~, dAccMinus] = EvalExtSphHarmExpInTargetFrame( ...
        dRSC_TB - dPerturb, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

    dJacSHE_TB(:, idxAxis) = (dAccPlus - dAccMinus) / (2.0 * dStep);

end

% Symmetrize the Jacobian and remove the trace to enforce the properties of the exterior SH acceleration Jacobian
dJacSHE_TB = 0.5 * (dJacSHE_TB + dJacSHE_TB.');
dJacSHE_TB = dJacSHE_TB - trace(dJacSHE_TB) / 3.0 * eye(3);

end
