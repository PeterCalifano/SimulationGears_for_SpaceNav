function [dPotentialPert, dAccPert_TB, dJacSHE_TB] = EvalExtSphHarmExpInTargetFrame( ...
    dPosSC_TB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
%% SIGNATURE
% [dPotentialPert, dAccPert_TB, dJacSHE_TB] = EvalExtSphHarmExpInTargetFrame( ...
%     dPosSC_TB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Canonical target-frame evaluator for the Exterior Spherical Harmonics
% Expansion model.
%
% This function returns the non-spherical potential and non-spherical
% acceleration in the target-body fixed frame using the repo-native
% unnormalized [Clm, Slm] storage convention.
%
% ACHTUNG: This function returns only the non-spherical contribution. The
% central-force term must be added separately if the total gravity field is
% required.
%
% Evaluate the spherical-harmonic potential with a Cartesian recurrence that
% remains regular at either pole. Derive acceleration and its analytical
% position Jacobian from the same potential.
% Request the third output for the Jacobian; two-output callers retain the
% first-order recurrence. Central gravity remains the caller's responsibility.
% Example: [dPotential, dAcceleration, dJacobian] = ...
%     EvalExtSphHarmExpInTargetFrame([2; 0; 0], uint32(2), zeros(4, 2), 1, 1);
% Output: Zero potential, acceleration and Jacobian for zero perturbing coefficients.
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_TB:        [3x1]     Position in the target-body fixed frame.
% ui32MaxDegree:    [1]       Maximum harmonic degree.
% dCSlmCoeffCols:   [Nl x 2]  Unnormalized [Clm, Slm] coefficient table.
% dGravParam:       [1]       Gravitational parameter [LU^3/TU^2].
% dBodyRadiusRef:   [1]       Reference radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialPert:   [1]       Non-spherical potential [LU^2/TU^2].
% dAccPert_TB:      [3x1]     Non-spherical acceleration in the target-body
%                             fixed frame [LU/TU^2].
% dJacSHE_TB                  (3,3) Acceleration position Jacobian [1/TU^2]; optional.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-04-2026    Pietro Califano     Add canonical target-frame SHE evaluator.
% 26-04-2026    Pietro Califano     Reuse shared spherical-to-Cartesian gradient mapping.
% 07-10-2026    Pietro Califano, Codex  Share regular Cartesian force and Hessian evaluation.
% 10-10-2026    Pietro Califano     Consolidate the Cartesian recurrence and optional Jacobian.
% 10-10-2026    Pietro Califano     Restore chronological interface history.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None; local scalar product rules propagate values, gradients and Hessians.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dPosSC_TB           (3,1) double {mustBeFinite, mustBeReal}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeFinite, mustBeReal}
    dGravParam          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
arguments (Output)
    dPotentialPert      (1,1) double
    dAccPert_TB         (3,1) double
    dJacSHE_TB          (3,3) double
end

%% Function code

% Reject the singular origin and return a zero perturbation below degree two.
dRadiusSquared = dot(dPosSC_TB, dPosSC_TB);
assert(dRadiusSquared > 0, 'EvalExtSphHarmExpInTargetFrame:ZeroPosition', ...
    'Position vector must be nonzero.');

dPotentialPert = 0;
dAccPert_TB = zeros(3, 1);
dJacSHE_TB = zeros(3, 3);
if ui32MaxDegree < uint32(2)
    return
end

ui32RequiredRows = ((ui32MaxDegree + uint32(1)) * (ui32MaxDegree + uint32(2))) / uint32(2) - uint32(2);
assert(size(dCSlmCoeffCols, 1) >= double(ui32RequiredRows), ...
    'EvalExtSphHarmExpInTargetFrame:InsufficientCoefficients', ...
    'Coefficient storage does not cover the requested degree.');

% Pack value and gradient; append the nine Hessian entries only for second-order callers.
bWithHessian = coder.const(nargout > 2);
dDerivativeCount = coder.const(4 + 9 * double(bWithHessian));

% Triangular storage inherits the coefficient array's fixed or bounded capacity.
% Avoid a compile-time size requirement when the shared RHS passes a bounded optional slice.
dBasis = complex(zeros(dDerivativeCount, size(dCSlmCoeffCols, 1) + 2));

% Recurrence multipliers R*x_i/r^2 (columns 1-3) and R^2/r^2 (column 4) with their derivatives.
dMultipliers = zeros(dDerivativeCount, 4);
dInverseRadiusSq = 1 / dRadiusSquared;
dGradInverseRadiusSq = -2 * dPosSC_TB * dInverseRadiusSq^2;
dHessInverseRadiusSq = -2 * eye(3) * dInverseRadiusSq^2 + ...
    8 * (dPosSC_TB * dPosSC_TB.') * dInverseRadiusSq^3;

for ui32Axis = uint32(1):uint32(3)
    dAxis = zeros(3, 1);
    dAxis(ui32Axis) = 1;

    dGradient = dBodyRadiusRef * (dAxis * dInverseRadiusSq + dPosSC_TB(ui32Axis) * dGradInverseRadiusSq);
    dHessian = dBodyRadiusRef * (dAxis * dGradInverseRadiusSq.' + dGradInverseRadiusSq * dAxis.' + ...
        dPosSC_TB(ui32Axis) * dHessInverseRadiusSq);

    dMultipliers(1:4, ui32Axis) = [dBodyRadiusRef * dPosSC_TB(ui32Axis) * dInverseRadiusSq; dGradient];
    if coder.const(bWithHessian)
        dMultipliers(5:13, ui32Axis) = dHessian(:);
    end
end

dMultipliers(1:4, 4) = dBodyRadiusRef^2 * [dInverseRadiusSq; dGradInverseRadiusSq];
if coder.const(bWithHessian)
    dMultipliers(5:13, 4) = dBodyRadiusRef^2 * dHessInverseRadiusSq(:);
end

% The degree-zero seed is R/r. Complex diagonals encode cosine and sine harmonics.
dSeedHessian = -dBodyRadiusRef * eye(3) * dInverseRadiusSq^(3/2) + ...
    3 * dBodyRadiusRef * (dPosSC_TB * dPosSC_TB.') * dInverseRadiusSq^(5/2);
dBasis(1:4, 1) = [dBodyRadiusRef * sqrt(dInverseRadiusSq); ...
                  -dBodyRadiusRef * dPosSC_TB * dInverseRadiusSq^(3/2)];
if coder.const(bWithHessian)
    dBasis(5:13, 1) = dSeedHessian(:);
end

dHorizontal = complex(dMultipliers(:, 1), dMultipliers(:, 2));
dPotentialDerivatives = complex(zeros(dDerivativeCount, 1));
ui32CoefficientRow = uint32(2);

% Advance the spherical-harmonic recurrence by degree and accumulate coefficient contributions.
for ui32Degree = uint32(1):ui32MaxDegree

    ui32CurrentStart  = ui32Degree * (ui32Degree + uint32(1)) / uint32(2) + uint32(1);
    ui32PreviousStart = (ui32Degree - uint32(1)) * ui32Degree / uint32(2) + uint32(1);

    if ~coder.target('MATLAB')
        % Fixed scalar products compile faster than variable-width order batches.
        % Keep the same recurrence and product-rule kernel in both execution layouts.
        for ui32Order = uint32(0):ui32Degree

            ui32Column = ui32CurrentStart + ui32Order;

            if ui32Order == ui32Degree
                % Sectorial term from the previous sectorial term.
                dBasis(:, ui32Column) = (2 * double(ui32Degree) - 1) * ...
                    MultiplyDerivatives_(dHorizontal, ...
                    dBasis(:, ui32PreviousStart + ui32Degree - uint32(1)), bWithHessian);
            else
                % Vertical recurrence from the two previous degrees of the same order.
                dLeading = (2 * double(ui32Degree) - 1) / double(ui32Degree - ui32Order);
                dBasis(:, ui32Column) = dLeading * MultiplyDerivatives_(dMultipliers(:, 3), ...
                    dBasis(:, ui32PreviousStart + ui32Order), bWithHessian);

                if ui32Degree >= ui32Order + uint32(2)
                    dTrailing = double(ui32Degree + ui32Order - uint32(1)) / double(ui32Degree - ui32Order);
                    ui32EarlierStart = (ui32Degree - uint32(2)) * (ui32Degree - uint32(1)) / uint32(2) + uint32(1);
                    dBasis(:, ui32Column) = dBasis(:, ui32Column) - dTrailing * ...
                        MultiplyDerivatives_(dMultipliers(:, 4), ...
                        dBasis(:, ui32EarlierStart + ui32Order), bWithHessian);
                end
            end

            if ui32Degree >= uint32(2)
                dCoefficient = complex(dCSlmCoeffCols(ui32CoefficientRow, 1), ...
                    -dCSlmCoeffCols(ui32CoefficientRow, 2));
                dPotentialDerivatives = dPotentialDerivatives + dCoefficient * dBasis(:, ui32Column);
                ui32CoefficientRow = ui32CoefficientRow + uint32(1);
            end
        end
        continue
    end

    % Batch all nonsectorial orders in one product-rule evaluation per recurrence term.
    ui32Orders = uint32(0):ui32Degree - uint32(1);
    ui32Columns = ui32PreviousStart + ui32Orders;
    dLeading = (2 * double(ui32Degree) - 1) ./ double(ui32Degree - ui32Orders);

    dPrevious = dBasis(:, ui32Columns);
    dCurrent = MultiplyDerivatives_(dMultipliers(:, 3), dPrevious, bWithHessian) .* dLeading;

    if ui32Degree >= uint32(2)
        ui32EarlierStart = (ui32Degree - uint32(2)) * (ui32Degree - uint32(1)) / uint32(2) + uint32(1);
        ui32TrailingColumns = ui32EarlierStart + ui32Orders(1:end-1);
        dTrailing = double(ui32Degree + ui32Orders(1:end-1) - uint32(1)) ./ ...
            double(ui32Degree - ui32Orders(1:end-1));

        dPrevious = dBasis(:, ui32TrailingColumns);
        dCurrent(:, 1:end-1) = dCurrent(:, 1:end-1) - ...
            MultiplyDerivatives_(dMultipliers(:, 4), dPrevious, bWithHessian) .* dTrailing;
    end

    dBasis(:, ui32CurrentStart + ui32Orders) = dCurrent;
    dBasis(:, ui32CurrentStart + ui32Degree) = (2 * double(ui32Degree) - 1) * ...
        MultiplyDerivatives_(dHorizontal, dBasis(:, ui32PreviousStart + ui32Degree - uint32(1)), bWithHessian);

    if ui32Degree >= uint32(2)
        % Re[(C-i*S)*(V+i*W)] preserves the existing positive sine convention.
        ui32Rows = ui32CoefficientRow:ui32CoefficientRow + ui32Degree;
        dCoefficients = complex(dCSlmCoeffCols(ui32Rows, 1), -dCSlmCoeffCols(ui32Rows, 2)).';
        dCurrent = dBasis(:, ui32CurrentStart:ui32CurrentStart + ui32Degree);

        dPotentialDerivatives = dPotentialDerivatives + sum(dCoefficients .* dCurrent, 2);
        ui32CoefficientRow = ui32CoefficientRow + ui32Degree + uint32(1);
    end
end

% Scale the accumulated terms to potential, acceleration and its position Jacobian.
dPotentialPert = (dGravParam / dBodyRadiusRef) * real(dPotentialDerivatives(1));
dAccPert_TB = (dGravParam / dBodyRadiusRef) * real(dPotentialDerivatives(2:4));
if coder.const(bWithHessian)
    dJacSHE_TB = (dGravParam / dBodyRadiusRef) * reshape(real(dPotentialDerivatives(5:13)), 3, 3);
end
end

function dProduct = MultiplyDerivatives_(dLeft, dRight, bWithHessian)
%% SIGNATURE
% dProduct = MultiplyDerivatives_(dLeft, dRight, bWithHessian)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply the exact scalar product rule through the requested derivative order.
% Rows hold [value; gradient(3); Hessian(9, column-major)].
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dLeft          (K,1) Packed derivatives of the left factor.
% dRight         (K,M) Packed derivatives of M right factors.
% bWithHessian   Constant selector of the second-order rows.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dProduct       (K,M) Packed derivatives of each product.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-10-2026  Pietro Califano, Codex  Share value, gradient and Hessian product rules.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dLeft         (:,1) double
    dRight        (:,:) double
    bWithHessian  (1,1) logical {coder.mustBeConst}
end
arguments (Output)
    dProduct      (:,:) double
end

dProduct = complex(zeros(size(dLeft, 1), size(dRight, 2)));
dProduct(1, :) = dLeft(1) * dRight(1, :);
dProduct(2:4, :) = dLeft(1) * dRight(2:4, :) + dLeft(2:4) * dRight(1, :);

if coder.const(bWithHessian)
    % H(fg) = f*H(g) + g*H(f) + grad(f)*grad(g)' + grad(g)*grad(f)'.
    ui32RowAxes = uint32([1; 2; 3; 1; 2; 3; 1; 2; 3]);
    ui32ColumnAxes = uint32([1; 1; 1; 2; 2; 2; 3; 3; 3]);
    dProduct(5:13, :) = dLeft(1) * dRight(5:13, :) + dLeft(5:13) * dRight(1, :) + ...
        dLeft(ui32RowAxes + 1) .* dRight(ui32ColumnAxes + 1, :) + ...
        dLeft(ui32ColumnAxes + 1) .* dRight(ui32RowAxes + 1, :);
end
end
