function dSamples = SampleCovarianceMatchedVectors(objRandomStream, dCovariance, ...
                                                    ui32NumSamples, enumDistribution)
%% SIGNATURE
% dSamples = SampleCovarianceMatchedVectors(objRandomStream, dCovariance, ...
%     ui32NumSamples, enumDistribution)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Draw zero-mean vectors with the configured covariance from a caller-owned random stream. Gaussian and uniform solid-
% ellipsoid distributions share one covariance factorization. A zero sample count validates the covariance and returns
% an empty sample matrix without advancing the stream. No global RNG state, seed derivation, retries, or provenance are
% owned by this generic routine.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% objRandomStream    (1,1) RandStream caller-owned deterministic stream
% dCovariance        (N,N) double finite symmetric positive-semidefinite covariance
% ui32NumSamples     (1,1) uint32 requested sample count; zero performs validation only
% enumDistribution   (1,1) EnumCovarianceSamplingDistribution probability law
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dSamples           (N,M) double samples stored column-wise
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RandStream, EnumCovarianceSamplingDistribution, chol, eig.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    objRandomStream (1,1) RandStream
    dCovariance (:,:) double {mustBeReal, mustBeFinite}
    ui32NumSamples (1,1) uint32
    enumDistribution (1,1) EnumCovarianceSamplingDistribution
end
arguments (Output)
    dSamples (:,:) double
end

dCovarianceFactor = FactorCovariance_(dCovariance);
ui32Dimension = uint32(size(dCovariance, 1));
if ui32NumSamples == uint32(0)
    dSamples = zeros(double(ui32Dimension), 0);
    return
end

switch enumDistribution
    case EnumCovarianceSamplingDistribution.GAUSSIAN
        dStandardSamples = randn(objRandomStream, double(ui32Dimension), double(ui32NumSamples));

    case EnumCovarianceSamplingDistribution.UNIFORM_SOLID_ELLIPSOID
        % A point uniform in the n-ball has covariance I/(n+2). The scale
        % below therefore makes the mapped solid-ellipsoid covariance exact.
        dUnitDirections = randn(objRandomStream, double(ui32Dimension), double(ui32NumSamples));
        dDirectionNorms = sqrt(sum(dUnitDirections.^2, 1));
        if any(dDirectionNorms == 0.0)
            error('SampleCovarianceMatchedVectors:DegenerateUniformDirection', ...
                'The caller-owned stream produced a zero direction.');
        end
        dUnitDirections = dUnitDirections ./ dDirectionNorms;
        dRadialFractions = rand(objRandomStream, 1, double(ui32NumSamples)).^ ...
            (1.0 / double(ui32Dimension));
        dStandardSamples = sqrt(double(ui32Dimension) + 2.0) * (dUnitDirections .* dRadialFractions);

    otherwise
        error('SampleCovarianceMatchedVectors:UnsupportedDistribution', ...
            'Unsupported covariance sampling distribution.');
end

dSamples = dCovarianceFactor * dStandardSamples;

end

function dCovarianceFactor = FactorCovariance_(dCovariance)
%% SIGNATURE
% dCovarianceFactor = FactorCovariance_(dCovariance)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate and factor one finite symmetric positive-semidefinite covariance of arbitrary nonzero square dimension.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dCovariance          (N,N) double configured covariance
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dCovarianceFactor    (N,N) double covariance square-root factor
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-08-2026  Pietro Califano, Codex     First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% chol, eig.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dCovariance (:,:) double {mustBeReal, mustBeFinite}
end
arguments (Output)
    dCovarianceFactor (:,:) double
end

if isempty(dCovariance)
    error('SampleCovarianceMatchedVectors:EmptyCovariance', ...
        'Covariance must have a nonzero square dimension.');
end
if size(dCovariance, 1) ~= size(dCovariance, 2)
    error('SampleCovarianceMatchedVectors:NonSquareCovariance', ...
        'Covariance must be square.');
end

dCovarianceScale = norm(dCovariance, 'fro');
dSymmetryTolerance = 1.0e-12 * dCovarianceScale;
if norm(dCovariance - transpose(dCovariance), 'fro') > dSymmetryTolerance
    error('SampleCovarianceMatchedVectors:NonSymmetricCovariance', ...
        'Covariance must be symmetric.');
end

dSymmetricCovariance = 0.5 * (dCovariance + transpose(dCovariance));
[dCovarianceFactor, dCholeskyFlag] = chol(dSymmetricCovariance, 'lower');
if dCholeskyFlag == 0.0
    return
end

[dEigenvectors, dEigenvalues] = eig(dSymmetricCovariance, 'vector');
dEigenvalueTolerance = 1.0e-12 * dCovarianceScale;
if any(dEigenvalues < -dEigenvalueTolerance)
    error('SampleCovarianceMatchedVectors:NonPositiveSemidefiniteCovariance', ...
        'Covariance must be positive semidefinite.');
end

dCovarianceFactor = dEigenvectors * diag(sqrt(max(dEigenvalues, 0.0)));

end
