function strProfile = GenerateGaussMarkovAccelSeq(dTimeBounds, ...
                                                   dSigmaAccel, ...
                                                   dTimeConst, ...
                                                   dTimeStep, ...
                                                   ui32Seed, ...
                                                   kwargs)
arguments
    dTimeBounds (1,2) double {mustBeFinite}
    dSigmaAccel (:,1) double {mustBeNonnegative}
    dTimeConst (:,1) double {mustBePositive}
    dTimeStep (1,1) double {mustBePositive}
    ui32Seed (1,1) uint32
end
arguments
    kwargs.dMeanAccel (3,1) double = zeros(3,1)
    kwargs.dInitialAccel (3,1) double = zeros(3,1)
    kwargs.bSampleInitialAccel (1,1) logical = true
    kwargs.charFrame (1,:) char = 'IN'
    kwargs.charSequenceModel (1,:) char = 'statistics_preserving'
end
%% SIGNATURE
% strProfile = GenerateGaussMarkovAccelSeq(dTimeBounds, dSigmaAccel, dTimeConst, dTimeStep, ui32Seed, kwargs)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generate a three-axis Gauss-Markov acceleration profile on a fixed stochastic support grid. The generated
% profile stores the standard-normal innovations so EvalGaussMarkovAccel can evaluate the same process at
% arbitrary RHS timestamps without drawing random numbers inside the dynamics function.
% Reference: Giordano, C. Characterization of Gauss–Markov stochastic sequences for mission analysis. 
% Astrodyn 8, 135–148 (2024). https://doi.org/10.1007/s42064-023-0183-3
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dTimeBounds:         (1,2) double   Ordered [initial, final] time bounds for the stochastic profile [TU].
% dSigmaAccel:         (1|3,1) double One-sigma stationary acceleration level, scalar or per inertial axis [LU/TU^2].
% dTimeConst:          (1|3,1) double Gauss-Markov correlation time, scalar or per inertial axis [TU].
% dTimeStep:           (1,1) double   Nominal support-grid spacing [TU].
% ui32Seed:            (1,1) uint32   Deterministic seed for the profile-specific random stream.
% kwargs.dMeanAccel:   (3,1) double   Mean residual acceleration in inertial-frame components [LU/TU^2].
% kwargs.dInitialAccel:(3,1) double   Initial acceleration when kwargs.bSampleInitialAccel is false [LU/TU^2].
% kwargs.bSampleInitialAccel:
%                       (1,1) logical If true, sample the initial acceleration from the stationary process.
% kwargs.charFrame:    (1,:) char     Frame label stored in the returned profile.
% kwargs.charSequenceModel:
%                       (1,:) char    Sequence-generation model; currently only 'statistics_preserving'.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strProfile:          (1,1) struct   Serializable stochastic profile consumed by EvalGaussMarkovAccel().
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-07-2026    Pietro Califano, Codex 5.5      Add deterministic Gauss-Markov residual acceleration profile
%                                               generation for max-fidelity truth dynamics.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RandStream() [MATLAB]
% -------------------------------------------------------------------------------------------------------------

assert(dTimeBounds(2) >= dTimeBounds(1), ...
    'GenerateGaussMarkovAccelSeq:InvalidTimeBounds', ...
    'dTimeBounds must be ordered as [t0, tf].');
assert(numel(dSigmaAccel) == 1 || numel(dSigmaAccel) == 3, ...
    'GenerateGaussMarkovAccelSeq:InvalidSigmaSize', ...
    'dSigmaAccel must be scalar or 3x1.');
assert(numel(dTimeConst) == 1 || numel(dTimeConst) == 3, ...
    'GenerateGaussMarkovAccelSeq:InvalidTimeConstSize', ...
    'dTimeConst must be scalar or 3x1.');
assert(strcmp(kwargs.charSequenceModel, 'statistics_preserving'), ...
    'GenerateGaussMarkovAccelSeq:UnsupportedSequenceModel', ...
    'Only statistics_preserving Gauss-Markov acceleration sequences are implemented.');

dSigmaAccel = ExpandToThree_(dSigmaAccel);
dTimeConst = ExpandToThree_(dTimeConst);
dMeanAccel = kwargs.dMeanAccel(:);

% Make timegrid for evaluation
dTimeGrid = dTimeBounds(1):dTimeStep:dTimeBounds(2);
if isempty(dTimeGrid)
    dTimeGrid = dTimeBounds(1);
end
if dTimeGrid(end) < dTimeBounds(2)
    dTimeGrid = [dTimeGrid, dTimeBounds(2)];
end

ui32NumSamples = uint32(numel(dTimeGrid));
ui32NumSegments = uint32(max(0, double(ui32NumSamples) - 1));
dAccelGrid = zeros(3, double(ui32NumSamples));
dStandardNormalInnovation = zeros(3, double(ui32NumSegments));

objStream = RandStream('mt19937ar', 'Seed', double(ui32Seed));

if kwargs.bSampleInitialAccel
    dAccelGrid(:,1) = dMeanAccel + dSigmaAccel .* randn(objStream, 3, 1);
else
    dAccelGrid(:,1) = kwargs.dInitialAccel(:);
end

% Compute stochastic acceleration realizations
for ui32Idx = 1:double(ui32NumSegments)

    dSegmentStep = dTimeGrid(ui32Idx + 1) - dTimeGrid(ui32Idx);
    dPhi = exp(-dSegmentStep ./ dTimeConst);
    dStandardNormalInnovation(:,ui32Idx) = randn(objStream, 3, 1);
    
    dAccelGrid(:,ui32Idx + 1) = dMeanAccel + ...
        dPhi .* (dAccelGrid(:,ui32Idx) - dMeanAccel) + ...
        dSigmaAccel .* sqrt(max(0.0, 1.0 - dPhi.^2)) .* dStandardNormalInnovation(:,ui32Idx);

end

strProfile = struct();
strProfile.dTimeGrid = dTimeGrid;
strProfile.dAccelGrid = dAccelGrid;
strProfile.dStandardNormalInnovation = dStandardNormalInnovation;
strProfile.dSigmaAccel = dSigmaAccel;
strProfile.dTimeConst = dTimeConst;
strProfile.dMeanAccel = dMeanAccel;
strProfile.dTimeStep = dTimeStep;
strProfile.ui32Seed = ui32Seed;
strProfile.charFrame = kwargs.charFrame;
strProfile.charSequenceModel = kwargs.charSequenceModel;
end

function dValue = ExpandToThree_(dValue)
% Expand scalar process parameter to three independent inertial axes.
if isscalar(dValue)
    dValue = repmat(dValue, 3, 1);
else
    dValue = dValue(:);
end
end
