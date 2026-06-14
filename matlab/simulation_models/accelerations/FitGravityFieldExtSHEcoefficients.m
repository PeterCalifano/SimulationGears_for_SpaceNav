function [dCSlmCoeffCols, strFitInfo] = FitGravityFieldExtSHEcoefficients( ...
    dSamplePos_TB, ...
    dSamplePotentialPert, ...
    dSampleAccPertTB, ...
    ui32MaxDegree, ...
    dGravParam, ...
    dBodyRadiusRef)%#codegen
arguments
    dSamplePos_TB           (3,:) double
    dSamplePotentialPert    (:,1) double
    dSampleAccPertTB        (3,:) double
    ui32MaxDegree           (1,1) uint32
    dGravParam              (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dBodyRadiusRef          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
%% PROTOTYPE
% [dCSlmCoeffCols, strFitInfo] = FitGravityFieldExtSHEcoefficients( ...
%     dSamplePos_TB, dSamplePotentialPert, dSampleAccPertTB, ...
%     ui32MaxDegree, dGravParam, dBodyRadiusRef)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Fits the repo-native exterior spherical harmonics coefficient storage
% from samples of the perturbative gravity field.
%
% The input field must already exclude the central point-mass term:
%   dSamplePotentialPert = U - mu / r
%   dSampleAccPertTB     = a + mu * rVec / r^3
%
% The solve is performed jointly on potential and acceleration through an
% overdetermined least-squares system, using the same unnormalized storage
% convention consumed by EvalExtSphHarmExpInTargetFrame().
%
% This function is agnostic to how samples were generated. Polyhedron-specific
% sample generation and adaptive shell refinement are owned by
% FitSpherHarmCoeffToPolyhedrGrav(), which calls this routine for the actual
% coefficient solve.
%
% ACHTUNG: Degree-1 storage is preserved in the returned table for format
% compatibility but kept fixed to zero. Likewise, Sl0 terms are fixed to
% zero because they do not contribute to the real-valued gravity field.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSamplePos_TB:         [3 x N]   Sample positions in target-body fixed frame.
% dSamplePotentialPert:  [N x 1]   Perturbative potential samples [LU^2/TU^2].
% dSampleAccPertTB:      [3 x N]   Perturbative acceleration samples [LU/TU^2].
% ui32MaxDegree:         [1]       Maximum harmonic degree to fit.
% dGravParam:            [1]       Gravitational parameter [LU^3/TU^2].
% dBodyRadiusRef:        [1]       Reference body radius [LU].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dCSlmCoeffCols:        [Nl x 2]  Unnormalized [Clm, Slm] coefficient table.
% strFitInfo:            struct    Solver diagnostics and fit residual metrics.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-04-2026    Pietro Califano     Add low-level field-to-SHE least-squares fitter.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildExtSphHarmExpBasis()
% MapSphericalGradientToCartesian()
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Input validation
if ui32MaxDegree < uint32(2)
    error('FitGravityFieldExtSHEcoefficients:InvalidMaxDegree', ...
        'The maximum degree must be at least 2.');
end

ui32NumSamples = uint32(size(dSamplePos_TB, 2));
if ui32NumSamples ~= uint32(size(dSampleAccPertTB, 2)) || ...
        ui32NumSamples ~= uint32(numel(dSamplePotentialPert))
    error('FitGravityFieldExtSHEcoefficients:SizeMismatch', ...
        'Position, potential, and acceleration samples must contain the same number of points.');
end

if any(~isfinite(dSamplePos_TB), 'all') || any(~isfinite(dSamplePotentialPert)) || any(~isfinite(dSampleAccPertTB), 'all')
    error('FitGravityFieldExtSHEcoefficients:InvalidSamples', ...
        'All sample arrays must contain finite values only.');
end

% Build the least-squares system for the joint potential and acceleration fit
ui32NumStorageCoefficients = (ui32MaxDegree + uint32(1)) * (ui32MaxDegree + uint32(2)) / uint32(2) - uint32(2);
ui32NumUnknowns = ui32MaxDegree * ui32MaxDegree + 2 * ui32MaxDegree - uint32(3);
ui32CoeffRowIds = zeros(double(ui32NumUnknowns), 1, 'uint32');
ui8CoeffColIds = zeros(double(ui32NumUnknowns), 1, 'uint8');

if 4 * double(ui32NumSamples) < double(ui32NumUnknowns)
    error('FitGravityFieldExtSHEcoefficients:UnderdeterminedSystem', ...
        'At least ceil(Nunknowns/4) sample points are required for the joint fit.');
end

dDesignMatrixRaw = zeros(4 * double(ui32NumSamples), double(ui32NumUnknowns));
dTargetVectorRaw = zeros(4 * double(ui32NumSamples), 1);
dRowScales = zeros(4 * double(ui32NumSamples), 1);

% Build regression matrix
for idSample = 1:double(ui32NumSamples)
    dPosSample = dSamplePos_TB(:, idSample);
    dRadius = norm(dPosSample);
    if dRadius <= 0.0
        error('FitGravityFieldExtSHEcoefficients:ZeroSamplePosition', ...
            'Sample positions must have strictly positive norm.');
    end

    dSCLat = asin(dPosSample(3) / dRadius);
    dSCLong = atan2(dPosSample(2), dPosSample(1));

    % Compute the unscaled potential and acceleration basis vectors for this sample
    [dPotentialBasis, dGradUBasis, ui32CoeffRowIdsSample, ui8CoeffColIdsSample] = BuildExtSphHarmExpBasis( ...
        dRadius, dSCLat, dSCLong, ui32MaxDegree, dGravParam, dBodyRadiusRef);
    dAccBasisTB = MapSphericalGradientToCartesian(dPosSample, dGradUBasis);

    if idSample == 1
        ui32CoeffRowIds = ui32CoeffRowIdsSample;
        ui8CoeffColIds = ui8CoeffColIdsSample;
    end

    % Scale rows to improve conditioning
    dPotRowScale = dRadius / dGravParam;
    dAccRowScale = dRadius^2 / dGravParam;

    % Store sample
    idRows = (4 * (idSample - 1) + 1):(4 * idSample);

    dDesignMatrixRaw(idRows(1), :) = dPotentialBasis;
    dDesignMatrixRaw(idRows(2:4), :) = dAccBasisTB;

    dTargetVectorRaw(idRows(1)) = dSamplePotentialPert(idSample);
    dTargetVectorRaw(idRows(2:4)) = dSampleAccPertTB(:, idSample);

    dRowScales(idRows(1)) = dPotRowScale;
    dRowScales(idRows(2:4)) = dAccRowScale;
end

% Apply normalization to design matrix and target vector
dDesignMatrix = dDesignMatrixRaw .* dRowScales;
dTargetVector = dTargetVectorRaw .* dRowScales;

dColumnNorms = vecnorm(dDesignMatrix, 2, 1);
if any(dColumnNorms <= 0.0)
    error('FitGravityFieldExtSHEcoefficients:DegenerateBasis', ...
        'The fit system contains one or more zero-norm columns.');
end

% Solve the least-squares problem through SVD
dDesignMatrixScaled = dDesignMatrix ./ dColumnNorms;
dSingularValues = svd(dDesignMatrixScaled, 'econ');

if isempty(dSingularValues) || dSingularValues(1) <= 0.0
    error('FitGravityFieldExtSHEcoefficients:DegenerateSystem', ...
        'Unable to build a valid fit system from the provided samples.');
end

% Apply minimum-norm regularization if the system is underdetermined after scaling
dRankTolerance = max(size(dDesignMatrixScaled)) * eps(dSingularValues(1));
ui32Rank = uint32(sum(dSingularValues > dRankTolerance));
bUsedLeastNorm = ui32Rank < ui32NumUnknowns;

if bUsedLeastNorm
    dCoeffScaled = lsqminnorm(dDesignMatrixScaled, dTargetVector);
else
    dCoeffScaled = dDesignMatrixScaled \ dTargetVector;
end

% Rescale coefficients back to original unnormalized basis and map to storage format
dCoeffUnknowns = dCoeffScaled ./ dColumnNorms.';
dCSlmCoeffCols = zeros(double(ui32NumStorageCoefficients), 2);

for idUnknown = 1:double(ui32NumUnknowns)
    dCSlmCoeffCols(ui32CoeffRowIds(idUnknown), ui8CoeffColIds(idUnknown)) = dCoeffUnknowns(idUnknown);
end

% Compute error metrics in physical units for diagnostics
dPredScaled = dDesignMatrixScaled * dCoeffScaled;
dResidualScaled = dPredScaled - dTargetVector;

dPredRaw = dDesignMatrixRaw * dCoeffUnknowns;
dPredRawBlocks = reshape(dPredRaw, 4, []);
dPredPotential = dPredRawBlocks(1, :).';
dPredAccTB = dPredRawBlocks(2:4, :).';

dTargetAccTB = dSampleAccPertTB.';
dTargetAccNorm = sqrt(mean(sum(dTargetAccTB.^2, 2)));
dPredAccNormErr = sqrt(mean(sum((dPredAccTB - dTargetAccTB).^2, 2)));
dTargetPotNorm = sqrt(mean(dSamplePotentialPert.^2));
dPredPotNormErr = sqrt(mean((dPredPotential - dSamplePotentialPert).^2));

strFitInfo = struct();
strFitInfo.ui32NumSamples = ui32NumSamples;
strFitInfo.ui32NumUnknowns = ui32NumUnknowns;
strFitInfo.ui32NumStorageCoefficients = ui32NumStorageCoefficients;
strFitInfo.ui32Rank = ui32Rank;
strFitInfo.bUsedLeastNorm = bUsedLeastNorm;
strFitInfo.dConditionNumber = dSingularValues(1) / dSingularValues(end);
strFitInfo.dScaledResidualRMS = norm(dResidualScaled) / sqrt(numel(dResidualScaled));
strFitInfo.dPotentialRMS = dPredPotNormErr;
strFitInfo.dPotentialRMSrel = dPredPotNormErr / max(dTargetPotNorm, eps(dTargetPotNorm));
strFitInfo.dAccRMS = dPredAccNormErr;
strFitInfo.dAccRMSrel = dPredAccNormErr / max(dTargetAccNorm, eps(dTargetAccNorm));
strFitInfo.ui32CoeffRowIds = ui32CoeffRowIds;
strFitInfo.ui8CoeffColIds = ui8CoeffColIds;

end
