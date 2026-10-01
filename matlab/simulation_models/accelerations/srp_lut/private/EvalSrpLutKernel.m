function [dForcePerPressure_SCB, dEffectiveCr, dTransverseForcePerPressure_SCB, ...
    dJacForcePerPressWrtSunPos_SCB, dJacCrWrtSunPos_SCB, ...
    dJacTransverseWrtSunPos_SCB, bDerivativeRegular] = ...
    EvalSrpLutKernel(dPosSCtoSun_SCB, strResponseLut, bIncludeTransverse, bComputeJacobian) %#codegen
%% SIGNATURE
% [dForcePerPressure_SCB, dEffectiveCr, dTransverseForcePerPressure_SCB, ...
%     dJacForcePerPressWrtSunPos_SCB, dJacCrWrtSunPos_SCB, dJacTransverseWrtSunPos_SCB, ...
%     bDerivativeRegular] = EvalSrpLutKernel(dPosSCtoSun_SCB, strResponseLut, ...
%     bIncludeTransverse, bComputeJacobian)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Share spherical interpolation and its analytical derivative between public
% force and linearization APIs. Differentiate the actual normalized query,
% bilinear scalar/vector interpolation and optional query-plane transverse projection.
% Select the cell containing the query, using its increasing-coordinate side
% at interior knots and the atan2 branch at the periodic seam. Return false
% for derivative regularity at knots and adjusted pole queries. Tilt lookup
% coordinates by a fixed tiny rotation near a pole and differentiate that
% rotation. Preserve the actual Sun direction in force/projection operations.
% Specialize bComputeJacobian at compile time to prune force-only work.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSCtoSun_SCB                   Nonzero spacecraft-to-Sun query in body coordinates.
% strResponseLut                    Validated immutable scalar/transverse fixed-capacity payload.
% bIncludeTransverse                Compile-time selection of transverse support.
% bComputeJacobian                  Compile-time selection of analytical partials.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dForcePerPressure_SCB             Body-frame force/pressure [m^2].
% dEffectiveCr                      Dimensionless parallel coefficient.
% dTransverseForcePerPressure_SCB   Optional transverse force/pressure [m^2].
% dJacForcePerPressWrtSunPos_SCB    Response partial w.r.t. the supplied query [m^2/query unit].
% dJacCrWrtSunPos_SCB               Scalar coefficient gradient [1/query unit].
% dJacTransverseWrtSunPos_SCB       Transverse response partial [m^2/query unit].
% bDerivativeRegular                True away from grid knots, seam and exact poles.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 29-09-2026  Pietro Califano, Codex gpt-6  Share LUT force and analytical partials.
% 29-09-2026  Pietro Califano, Codex gpt-6  Regularize pole lookup without random state.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify query frames, interpolation and derivative steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ValidateSrpResponseLut (host-side preparation).
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dPosSCtoSun_SCB (3, 1) double
    strResponseLut (1, 1) struct
    bIncludeTransverse (1, 1) logical {coder.mustBeConst}
    bComputeJacobian (1, 1) logical {coder.mustBeConst}
end

arguments (Output)
    dForcePerPressure_SCB (3, 1) double
    dEffectiveCr (1, 1) double
    dTransverseForcePerPressure_SCB (3, 1) double
    dJacForcePerPressWrtSunPos_SCB (3, 3) double
    dJacCrWrtSunPos_SCB (1, 3) double
    dJacTransverseWrtSunPos_SCB (3, 3) double
    bDerivativeRegular (1, 1) logical
end

% Guard query and active counts without scanning or copying immutable arrays.
assert(all(isfinite(dPosSCtoSun_SCB)), 'EvaluateSrpResponseLut:InvalidDirection', ...
    'Supply finite spacecraft-to-Sun direction components.');
assert(strResponseLut.ui32AzimuthCount >= 3 && ...
       strResponseLut.ui32AzimuthCount <= size(strResponseLut.dAzimuth, 2) && ...
       strResponseLut.ui32ElevationCount >= 3 && ...
       strResponseLut.ui32ElevationCount <= size(strResponseLut.dElevation, 2), ...
       'EvaluateSrpResponseLut:InvalidCount', 'Supply populated counts within fixed capacity.');

% Scale before normalization to avoid overflow for large displacement components.
dSunPosScale = max(abs(dPosSCtoSun_SCB));
assert(dSunPosScale > 0, 'EvaluateSrpResponseLut:ZeroDirection', ...
    'Supply a nonzero spacecraft-to-Sun direction.');

dScaledSunPos_SCB = dPosSCtoSun_SCB / dSunPosScale;
dScaledSunPosNorm = norm(dScaledSunPos_SCB);
dSunDir_SCB = dScaledSunPos_SCB / dScaledSunPosNorm;

% Select a deterministic off-axis lookup inside a tiny cap around either pole.
% Use the golden-angle meridian to avoid grid-aligned perturbations. Keep the
% cap smaller than the tilt so the adjusted query cannot cancel onto the axis.
dPoleTiltAngle = 1e-8;  % Radians; independent of table spacing and query magnitude.
dLookupEquatorialRadius = hypot(dSunDir_SCB(1), dSunDir_SCB(2));
bPoleAdjusted = dLookupEquatorialRadius < dPoleTiltAngle / 4;
dLookupRotation = eye(3);
dLookupSunDir = dSunDir_SCB;

if bPoleAdjusted
    dPoleTiltAxis = [-0.6754902942615238; -0.7373688780783197; 0];
    dPoleTiltAxisSkew = [0, 0, dPoleTiltAxis(2); 0, 0, -dPoleTiltAxis(1); ...
                        -dPoleTiltAxis(2), dPoleTiltAxis(1), 0];
    dLookupRotation = eye(3) + sin(dPoleTiltAngle) * dPoleTiltAxisSkew + ...
        (1 - cos(dPoleTiltAngle)) * (dPoleTiltAxisSkew * dPoleTiltAxisSkew);

    dLookupSunDir = dLookupRotation * dSunDir_SCB;
    dLookupEquatorialRadius = hypot(dLookupSunDir(1), dLookupSunDir(2));
end

% Use atan2 for elevation to retain precision close to either pole.
dAzimuth = atan2d(dLookupSunDir(2), dLookupSunDir(1));
dElevation = atan2d(dLookupSunDir(3), dLookupEquatorialRadius);
dAzimuthStep = strResponseLut.dAzimuth(2) - strResponseLut.dAzimuth(1);
dElevationStep = strResponseLut.dElevation(2) - strResponseLut.dElevation(1);

% Select the increasing-coordinate cell at a knot and clamp boundary cells.
dAzimuthGridIndex = (dAzimuth - strResponseLut.dAzimuth(1)) / dAzimuthStep;
dElevationGridIndex = (dElevation - strResponseLut.dElevation(1)) / dElevationStep;
ui32AzimuthCell = uint32(max(1, min(double(strResponseLut.ui32AzimuthCount) - 1, ...
    floor(dAzimuthGridIndex) + 1)));
ui32ElevationCell = uint32(max(1, min(double(strResponseLut.ui32ElevationCount) - 1, ...
    floor(dElevationGridIndex) + 1)));

dAzimuthWeight = max(0, min(1, dAzimuthGridIndex - (double(ui32AzimuthCell) - 1)));
dElevationWeight = max(0, min(1, dElevationGridIndex - (double(ui32ElevationCell) - 1)));
dElevationComplement = 1 - dElevationWeight;

% Retain the small pole-side weight without subtracting nearly equal angles.
if bPoleAdjusted
    dPoleWeight = atan2d(dLookupEquatorialRadius, abs(dLookupSunDir(3))) / dElevationStep;
    if dLookupSunDir(3) > 0
        ui32ElevationCell = strResponseLut.ui32ElevationCount - 1;
        dElevationWeight = 1 - dPoleWeight;
        dElevationComplement = dPoleWeight;
    else
        ui32ElevationCell = uint32(1);
        dElevationWeight = dPoleWeight;
        dElevationComplement = 1 - dPoleWeight;
    end
end

dBilinearWeights = [dElevationComplement * (1 - dAzimuthWeight), ...
                   dElevationWeight * (1 - dAzimuthWeight), ...
                   dElevationComplement * dAzimuthWeight, ...
                   dElevationWeight * dAzimuthWeight];

% Read the scalar corners once and form the independent Sun-parallel force.
dCrLowElevLowAz = strResponseLut.dEffectiveCr(ui32ElevationCell, ui32AzimuthCell);
dCrHighElevLowAz = strResponseLut.dEffectiveCr(ui32ElevationCell + 1, ui32AzimuthCell);
dCrLowElevHighAz = strResponseLut.dEffectiveCr(ui32ElevationCell, ui32AzimuthCell + 1);
dCrHighElevHighAz = strResponseLut.dEffectiveCr(ui32ElevationCell + 1, ui32AzimuthCell + 1);
dEffectiveCr = dBilinearWeights(1) * dCrLowElevLowAz + ...
               dBilinearWeights(2) * dCrHighElevLowAz + ...
               dBilinearWeights(3) * dCrLowElevHighAz + ...
               dBilinearWeights(4) * dCrHighElevHighAz;

dReferenceArea = strResponseLut.dReferenceArea_m2;
dForcePerPressure_SCB = -dSunDir_SCB * dEffectiveCr * dReferenceArea;

% Initialize optional outputs and identify cells without a unique derivative.
dTransverseForcePerPressure_SCB = zeros(3, 1);
dJacForcePerPressWrtSunPos_SCB = zeros(3, 3);
dJacCrWrtSunPos_SCB = zeros(1, 3);
dJacTransverseWrtSunPos_SCB = zeros(3, 3);
bDerivativeRegular = ~bPoleAdjusted && dLookupEquatorialRadius > 64 * eps && ...
    dAzimuthWeight > 64 * eps && dAzimuthWeight < 1 - 64 * eps && ...
    dElevationWeight > 64 * eps && dElevationWeight < 1 - 64 * eps;

% Differentiate the original query, retaining its displacement scale.
dJacSunDirWrtSunPos_SCB = zeros(3, 3);
dJacAzimuthWrtSunPos_SCB = zeros(1, 3);
dJacElevationWrtSunPos_SCB = zeros(1, 3);

if bComputeJacobian
    dInvSunPosNorm = (1 / dSunPosScale) / dScaledSunPosNorm;
    dJacSunDirWrtSunPos_SCB = (eye(3) - dSunDir_SCB * dSunDir_SCB.') * dInvSunPosNorm;

    % Chain lookup angles through the fixed pole rotation when it is applied.
    if dLookupEquatorialRadius > 64 * eps

        dJacAzimuthWrtSunPos_SCB = (180 / pi) * [-dLookupSunDir(2), dLookupSunDir(1), 0] / ...
            dLookupEquatorialRadius^2 * dInvSunPosNorm;

        dJacElevationWrtSunPos_SCB = (180 / pi) * ...
            [-dLookupSunDir(3) * dLookupSunDir(1) / dLookupEquatorialRadius, ...
             -dLookupSunDir(3) * dLookupSunDir(2) / dLookupEquatorialRadius, ...
             dLookupEquatorialRadius] * dInvSunPosNorm;

        if bPoleAdjusted
            dJacAzimuthWrtSunPos_SCB = dJacAzimuthWrtSunPos_SCB * dLookupRotation;
            dJacElevationWrtSunPos_SCB = dJacElevationWrtSunPos_SCB * dLookupRotation;
        end
    end

    % Combine cell slopes with angular partials before differentiating the force direction.
    dJacCrWrtAzimuth = (dElevationComplement * (dCrLowElevHighAz - dCrLowElevLowAz) + ...
        dElevationWeight * (dCrHighElevHighAz - dCrHighElevLowAz)) / dAzimuthStep;
    dJacCrWrtElevation = ((1 - dAzimuthWeight) * (dCrHighElevLowAz - dCrLowElevLowAz) + ...
        dAzimuthWeight * (dCrHighElevHighAz - dCrLowElevHighAz)) / dElevationStep;

    dJacCrWrtSunPos_SCB = dJacCrWrtAzimuth * dJacAzimuthWrtSunPos_SCB + ...
                         dJacCrWrtElevation * dJacElevationWrtSunPos_SCB;

    dJacForcePerPressWrtSunPos_SCB = -dReferenceArea * ...
        (dEffectiveCr * dJacSunDirWrtSunPos_SCB + dSunDir_SCB * dJacCrWrtSunPos_SCB);
end

if bIncludeTransverse
    % Reuse the scalar weights to interpolate nodal transverse samples.
    dTransLowElevLowAz_SCB = ...
        strResponseLut.dTransverseForcePerPressure(:, ui32ElevationCell, ui32AzimuthCell);
    dTransHighElevLowAz_SCB = ...
        strResponseLut.dTransverseForcePerPressure(:, ui32ElevationCell + 1, ui32AzimuthCell);
    dTransLowElevHighAz_SCB = ...
        strResponseLut.dTransverseForcePerPressure(:, ui32ElevationCell, ui32AzimuthCell + 1);
    dTransHighElevHighAz_SCB = ...
        strResponseLut.dTransverseForcePerPressure(:, ui32ElevationCell + 1, ui32AzimuthCell + 1);
    dInterpTransverse_SCB = dBilinearWeights(1) * dTransLowElevLowAz_SCB + ...
                           dBilinearWeights(2) * dTransHighElevLowAz_SCB + ...
                           dBilinearWeights(3) * dTransLowElevHighAz_SCB + ...
                           dBilinearWeights(4) * dTransHighElevHighAz_SCB;

    % Restore query-plane orthogonality without mixing in interpolated radial force.
    dParallelTransverse = dot(dInterpTransverse_SCB, dSunDir_SCB);
    dTransverseForcePerPressure_SCB = dInterpTransverse_SCB - dSunDir_SCB * dParallelTransverse;
    dForcePerPressure_SCB = dForcePerPressure_SCB + dTransverseForcePerPressure_SCB;

    if bComputeJacobian
        % Differentiate the transverse interpolation and query-plane projection.
        dJacTransverseWrtAz_SCB = ...
            (dElevationComplement * (dTransLowElevHighAz_SCB - dTransLowElevLowAz_SCB) + ...
             dElevationWeight * (dTransHighElevHighAz_SCB - dTransHighElevLowAz_SCB)) / dAzimuthStep;
        dJacTransverseWrtElev_SCB = ...
            ((1 - dAzimuthWeight) * (dTransHighElevLowAz_SCB - dTransLowElevLowAz_SCB) + ...
             dAzimuthWeight * (dTransHighElevHighAz_SCB - dTransLowElevHighAz_SCB)) / dElevationStep;

        dJacInterpTransverse_SCB = dJacTransverseWrtAz_SCB * dJacAzimuthWrtSunPos_SCB + ...
                                  dJacTransverseWrtElev_SCB * dJacElevationWrtSunPos_SCB;
        dTransverseProjector_SCB = eye(3) - dSunDir_SCB * dSunDir_SCB.';
        dJacTransverseWrtSunPos_SCB = dTransverseProjector_SCB * dJacInterpTransverse_SCB - ...
            (dParallelTransverse * eye(3) + dSunDir_SCB * dInterpTransverse_SCB.') * dJacSunDirWrtSunPos_SCB;
        dJacForcePerPressWrtSunPos_SCB = dJacForcePerPressWrtSunPos_SCB + dJacTransverseWrtSunPos_SCB;
    end
end
end
