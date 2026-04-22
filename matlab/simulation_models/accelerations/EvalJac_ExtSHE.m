function dJacSHE_TB = EvalJac_ExtSHE(dRSC_TB, ...
    ui32MaxDegree, ...
    dCSlmCoeffCols, ...
    dGravParam, ...
    dBodyRadiusRef) %#codegen
arguments
    dRSC_TB             (3,1) double {mustBeReal, mustBeFinite}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeReal, mustBeFinite}
    dGravParam          (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
end
%% PROTOTYPE
% dJacSHE_TB = EvalJac_ExtSHE(dRSC_TB, ui32MaxDegree, dCSlmCoeffCols, ...
%                             dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compute the 3x3 Jacobian of the non-spherical Exterior SHE acceleration.
%
% This routine evaluates the Jacobian of the acceleration returned by
% ExtSHE_AccTB with a central finite-difference stencil. The result is then
% projected onto the physically admissible subspace for an exterior gravity
% potential: symmetric and trace-free.
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% ----------------------------------------------------------------------------------------------------
%% INPUTS
% dRSC_TB:
%   [3x1] spacecraft position expressed in the target-body-fixed frame.
% ui32MaxDegree:
%   maximum spherical-harmonics degree. For degree < 2 the non-spherical
%   acceleration is identically zero and this function returns a zero 3x3.
% dCSlmCoeffCols:
%   [N x 2] unnormalized [Clm, Slm] coefficient column pairs used by
%   ExtSHE_AccTB. The minimum required number of rows is:
%       ((lMax + 1) * (lMax + 2)) / 2 - 2    for lMax >= 2
%   which corresponds to the ExtSHE storage convention starting at (1,1).
%   Extra rows are ignored.
% dGravParam:
%   body gravitational parameter.
% dBodyRadiusRef:
%   body reference radius used by the SHE model and by the finite-difference
%   step-size heuristic.
%
%% OUTPUTS
% dJacSHE_TB:
%   [3x3] Jacobian da/dr of the non-spherical acceleration in the same
%   target-body-fixed frame.
% ----------------------------------------------------------------------------------------------------
%% NOTES
% 1) The model is intended for exterior-field use, consistent with
%    ExtSHE_AccTB.
% 2) The Jacobian of a conservative gravity field is the Hessian of the
%    potential, so in free space it should be symmetric and have zero trace
%    (Laplace equation). The two projections below are not part of the
%    finite-difference formula itself; they are numerical cleanup steps that
%    remove small symmetry/trace violations caused by truncation and
%    round-off.
% ----------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ExtSHE_AccTB()
% ----------------------------------------------------------------------------------------------------

dPosNorm = norm(dRSC_TB);

if dPosNorm <= 0.0
    error('EvalJac_ExtSHE:ZeroPosition', ...
        'Position vector must be non-zero.');
end

if ui32MaxDegree >= uint32(2)

    ui32RequiredRows = ((ui32MaxDegree + uint32(1)) * (ui32MaxDegree + uint32(2))) ...
        / uint32(2) - uint32(2);

    if size(dCSlmCoeffCols, 1) < double(ui32RequiredRows)
        error('EvalJac_ExtSHE:InsufficientCoefficients', ...
            ['dCSlmCoeffCols has %d rows, but degree %d requires at least %d ' ...
            'rows in ExtSHE column-pair format.'], ...
            size(dCSlmCoeffCols, 1), double(ui32MaxDegree), double(ui32RequiredRows));
    end
end

% Initialize output
dStep = 1.0e-6 * max(dPosNorm, dBodyRadiusRef);
dJacSHE_TB = zeros(3, 3);

for idxAxis = 1:3

    % Evaluate accel for idx axis
    dPerturb = zeros(3, 1);
    dPerturb(idxAxis) = dStep;

    % Compute +/- perturbed values
    dAccPlus = ExtSHE_AccTB(dRSC_TB + dPerturb, ui32MaxDegree, ...
        dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

    dAccMinus = ExtSHE_AccTB(dRSC_TB - dPerturb, ui32MaxDegree, ...
        dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

    % Approximate derivative
    dJacSHE_TB(:, idxAxis) = (dAccPlus - dAccMinus) / (2.0 * dStep);
end

% The exact Jacobian is symmetric because it is the Hessian of the
% potential. This averages out the small antisymmetric part introduced by
% finite differencing.
dJacSHE_TB = 0.5 * (dJacSHE_TB + dJacSHE_TB.');

% In source-free space the Hessian should also have zero trace. This line
% removes only the isotropic numerical error on the diagonal. It is a
% projection onto the trace-free subspace, not a new physical model term.
dJacSHE_TB = dJacSHE_TB - trace(dJacSHE_TB) / 3.0 * eye(3);
end
