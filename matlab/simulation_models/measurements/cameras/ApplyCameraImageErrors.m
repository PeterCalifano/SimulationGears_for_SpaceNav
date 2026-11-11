function [varImgPostProcess, fLinearDetectorImage, dReadNoiseSigmaNorm, ...
          dDetectorNormalizationLevels] = ...
        ApplyCameraImageErrors(varGrayImg, objSettings)
%% SIGNATURE
% [varImgPostProcess, fLinearDetectorImage, dReadNoiseSigmaNorm, dDetectorNormalizationLevels] = ...
%     ApplyCameraImageErrors(varGrayImg, objSettings)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Apply the established hosted camera error model in digital-number units:
% blur, Poisson shot noise, white noise, bloom, radiation spots, dead pixels/
% columns and gamma correction, in that order. The caller owns the MATLAB RNG;
% this routine never seeds it. Disabled effects consume no random draws.
% The selected shot backend is fixed per trial. CUDA derives an exposure key
% deterministically from the caller's RNG; switching backend changes the realization
% and subsequent CPU RNG state, while repeatability is retained within a backend.
% Historical relative-noise aliases remain accepted at this sensor boundary.
% Sample Poisson charge at blurred_DN * dGainElectronsPerDN, then return to
% floating-point DN before independent additive Gaussian noise. Clip the
% combined detector signal once to [0,dNUM_LEVELS], before later effects and
% final datatype conversion. Preview gain is not a radiometric calibration.
% The second output preserves the signed, normalized signal before clipping
% and later nonlinear effects; the third reports normalized Gaussian read noise.
% Example: objSettings = SImageErrorsPipelineSettings(dGammaValue=2);
%          ui8Image = ApplyCameraImageErrors(uint8(128*ones(4)), objSettings);
% Output: a 4-by-4 uint8 image whose entries are 64.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% varGrayImg  Numeric grayscale image in DN.
% objSettings SImageErrorsPipelineSettings retaining legacy sensor field names.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% varImgPostProcess       Same shape, selected output type (input type by default).
% fLinearDetectorImage    Signed single-precision detector signal normalized by
%                         resolved dNUM_LEVELS, before clipping and later effects.
% dReadNoiseSigmaNorm     Resolved normalized standard deviation of Gaussian read noise.
% dDetectorNormalizationLevels Positive resolved DN denominator for the signed signal.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 20-05-2024  Pietro Califano  First generic blur and white-noise model.
% 06-03-2025  Pietro Califano  Add error-source models.
% 16-01-2026  Pietro Califano  Revise image error pipeline.
% 12-10-2026  Pietro Califano, Codex gpt-6.1  Move sensor ownership; repair gamma,
%            dead-column bounds, bloom helper calls and odd Gaussian support.
% 18-10-2026  Pietro Califano, Codex gpt-6.1  Sample charge with explicit gain;
%            retain shot excursions until after independent Gaussian noise.
% 19-10-2026  Pietro Califano, Codex GPT-6  Expose signed detector signal and read-noise scale.
% 19-10-2026  Pietro Califano, Codex GPT-6  Report the resolved signal normalization scale.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; prefixes match the stored class.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Image Processing Toolbox; Statistics Toolbox for active Poisson effects.
% -------------------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    varGrayImg (:,:) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative}
    objSettings (1,1) SImageErrorsPipelineSettings = SImageErrorsPipelineSettings()
end
arguments (Output)
    varImgPostProcess (:,:) {mustBeNumeric, mustBeReal, mustBeFinite}
    fLinearDetectorImage (:,:) single {mustBeReal, mustBeFinite}
    dReadNoiseSigmaNorm (1,1) double {mustBeReal, mustBeFinite, mustBeNonnegative}
    dDetectorNormalizationLevels (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
end

% Preserve value settings while resolving legacy aliases and automatic levels.
objErrorParams = objSettings;
objOutputSettings = objSettings;
if strcmpi(objOutputSettings.charOutputDataType, "input")
    charInputDatatype = class(varGrayImg);
    objOutputSettings.charOutputDataType = charInputDatatype;
else
    % Use output datatype if not double or float
    assert( objErrorParams.dNUM_LEVELS > 0 || ~any(objOutputSettings.charOutputDataType == ["single", "double"]), ...
        'ERROR: cannot automatically determine maximum number of levels based input or output types. Please specify as dNumLevels.')
    charInputDatatype = objOutputSettings.charOutputDataType;
end

dImgPostProcess = cast(varGrayImg, 'double');

% Determine number of levels based on the output datatype
if objErrorParams.dNUM_LEVELS == 0
    switch charInputDatatype
        case "uint8"
            objErrorParams.dNUM_LEVELS = 255;
        case "uint16"
            objErrorParams.dNUM_LEVELS = 65535;
        case "uint32"
            objErrorParams.dNUM_LEVELS = 4294967295;
        case "uint64"
            objErrorParams.dNUM_LEVELS = 18446744073709551615;
    end
end

dNUM_LEVELS = objErrorParams.dNUM_LEVELS;
fprintf('Adding error sources to image using %d intensity level...\n', dNUM_LEVELS);
assert(dNUM_LEVELS >= 1, 'ERROR: dNUM_LEVELS must be a positive integer number.')
dDetectorNormalizationLevels = dNUM_LEVELS;

% Resolve the DN-relative white-noise sigma from the legacy aliases
dSigmaDigitNumRelGenericWN = objErrorParams.dSigmaDigitNumRelGenericWN;
if dSigmaDigitNumRelGenericWN == 0 && objErrorParams.dSigmaGenericWNRel > 0
    dSigmaDigitNumRelGenericWN = objErrorParams.dSigmaGenericWNRel;
end
if dSigmaDigitNumRelGenericWN == 0 && objErrorParams.dSigmaGenericWN > 0
    dSigmaDigitNumRelGenericWN = sqrt(objErrorParams.dSigmaGenericWN ./ dNUM_LEVELS);
end

%% Hardware sources
%%% SCENE
% Motion blur filter
% motionBlurFilter = fspecial("motion"); % Create filter
% imgPostProcess = conv2(imgPostProcess, motionBlurFilter); % Apply filter

%%% OPTICS
% Step 1: Gaussian filter blur 2D convolution
% By default, imgaussfilt uses a square Gaussian kernel
if objErrorParams.dSigmaGenericBlur > 0

    dImgPostProcess = imgaussfilt(dImgPostProcess, ...
                                  objErrorParams.dSigmaGenericBlur, ...
                                  "FilterSize", objErrorParams.dGenericBlurFilterSize);

end

%%% DETECTOR
% Convert blurred preview DN to expected charge, then retain fractional DN.
if objErrorParams.bAddShotNoise == true
    dGainElectronsPerDN = objErrorParams.dGainElectronsPerDN;
    dExpectedElectrons = dImgPostProcess * dGainElectronsPerDN;
    dImgPostProcess = SampleCameraShotCharge(dExpectedElectrons, ...
        objErrorParams.charShotNoiseBackend) / dGainElectronsPerDN;
end

% Read noise is independent of charge; do not saturate the shot draw first.
if objErrorParams.dMeanGenericWN ~= 0 || dSigmaDigitNumRelGenericWN > 0
    dImgPostProcess = dImgPostProcess + objErrorParams.dMeanGenericWN;
    if dSigmaDigitNumRelGenericWN > 0
        dImgPostProcess = dImgPostProcess + dNUM_LEVELS * ...
            dSigmaDigitNumRelGenericWN * randn(size(dImgPostProcess));
    end
end

% Retain signed outputs only when requested so legacy preview calls keep their cost.
if nargout >= 2
    fLinearDetectorImage = single(dImgPostProcess ./ dNUM_LEVELS);
end
if nargout >= 3
    dReadNoiseSigmaNorm = 0;
    if dSigmaDigitNumRelGenericWN > 0
        dReadNoiseSigmaNorm = dSigmaDigitNumRelGenericWN;
    end
end

% Preserve detector output bounds without clipping fractional intermediates.
if objErrorParams.bAddShotNoise || objErrorParams.dMeanGenericWN ~= 0 || ...
        dSigmaDigitNumRelGenericWN > 0
    dImgPostProcess = max(0, min(dImgPostProcess, dNUM_LEVELS));
end

% Detector defects follow: blooming, radiation spots, dead pixels and dead columns.
% Blooming and radiation saturate pixels, rows or columns with a given probability
% and Poisson-distributed occurrence.
% Dead pixels are randomly chosen pixels set to 0.
% Dead buckets are randomly chosen whole columns set to 0.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EXPERIMENTAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Random bright spots

% Blooming (i.e., saturate clusters of pixels or entire rows/columns with masked blurring effect)
% Model occurrence using a Poisson distribution if desired.
if objErrorParams.dBloomingProbability > 0 && objErrorParams.dBloomPoissonMean > 0

    % Auxiliary params
    dBloomingProbability    = objErrorParams.dBloomingProbability;
    dLambdaBloom            = objErrorParams.dBloomPoissonMean;
    charBloomType           = objErrorParams.charBloomType;
    dPartialBloomFraction   = objErrorParams.dPartialBloomFraction;

    switch lower(charBloomType)
        case 'blob'

            % Copy image before processing
            dBeforeBloomImg = dImgPostProcess;

            % Define a brightness threshold at 0.98 of the full scale
            dThreshold = 0.98 * dNUM_LEVELS;

            % Identify all pixels above this threshold
            bBrightMask = (dImgPostProcess > dThreshold);

            % Optionally combine with Poisson events
            dPoissonMatrix = poissrnd(dLambdaBloom, size(dImgPostProcess));

            % Optionally keep a probability check as well
            dRandMatrix = rand(size(dImgPostProcess));

            % Final bloom mask: bright + random chance + Poisson
            % If you want guaranteed blooming for bright pixels, omit the random checks
            bRadiationBrightMask = bBrightMask & (dRandMatrix < dBloomingProbability) & (dPoissonMatrix > 0);

            % Dilate blooming mask
            objStrel = strel('disk', 2);
            bRadiationBrightMask = imdilate(bRadiationBrightMask, objStrel);

            % Set pixels to mask
            dImgPostProcess(bRadiationBrightMask) = dNUM_LEVELS;

            % Apply blur to bloomed pixels
            [dImgPostProcess] = ApplyBlurToBlooming_(dImgPostProcess, dBeforeBloomImg);

            % Clamp intensities to [0, dNUM_LEVELS]
            dImgPostProcess = max(0, min(dImgPostProcess, dNUM_LEVELS));

        otherwise

            assert(dPartialBloomFraction > 0, 'ERROR: required row/col blooming but dPartialBloomFraction default value is 0!')
            % Run simulation model for partial row/col blooming with gaussian roll-off
            dImgPostProcess = BloomBrightRowsCols_(dImgPostProcess, ...
                                                   dBloomingProbability, ...
                                                   dBloomingProbability, ...
                                                   dLambdaBloom, ...
                                                   dLambdaBloom, ...
                                                   dPartialBloomFraction, ...
                                                   dPartialBloomFraction, ...
                                                   dNUM_LEVELS);
    end
end

if objErrorParams.dBrightSpotsProbability > 0 && objErrorParams.dBrightSpotsPoissonRate > 0

    % Simulate bright spots from radiation hitting single pixels
    dRandMatrix = rand(size(dImgPostProcess));
    dPoissonMatrix = poissrnd(objErrorParams.dBrightSpotsPoissonRate, size(dImgPostProcess));
    bRadiationBrightMask = (dRandMatrix < objErrorParams.dBrightSpotsProbability) & (dPoissonMatrix > 0);

    dImgPostProcess(bRadiationBrightMask) = dNUM_LEVELS;
end

% Dead pixels
if objErrorParams.dDeadPixelPerc > 0
    % Clip values to [0, 1]
    objErrorParams.dDeadPixelPerc = max(0, objErrorParams.dDeadPixelPerc);
    objErrorParams.dDeadPixelPerc = min(1, objErrorParams.dDeadPixelPerc);

    % Get number of dead pixels
    dNumOfDeadPix = round(objErrorParams.dDeadPixelPerc * numel(dImgPostProcess));

    % Randomly pick pixels
    dRandPixIdx = randperm(numel(dImgPostProcess), dNumOfDeadPix);
    dImgPostProcess(dRandPixIdx) = 0.0;
end

% Dead buckets (set entire columns to zero)
if objErrorParams.dDeadBucketFraction > 0

    [~, dNumCols] = size(dImgPostProcess);

    % Clip values to [0, 1]
    objErrorParams.dDeadBucketFraction = max(0, objErrorParams.dDeadBucketFraction);
    objErrorParams.dDeadBucketFraction = min(1, objErrorParams.dDeadBucketFraction);

    % Choose how many columns to set
    dNumDeadCols = round(objErrorParams.dDeadBucketFraction * dNumCols);

    % Randomly pick which columns
    dDeadColsIdx = randperm(dNumCols, dNumDeadCols);

    % Zero them out
    dImgPostProcess(:, dDeadColsIdx) = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EXPERIMENTAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Software sources
% Gamma correction
% imadjust(I,[low_in; high_in],[low_out; high_out],gamma)
% If only gamma is provided, we can pass empty [] for the other params.
if objErrorParams.dGammaValue > 0 && objErrorParams.dNUM_LEVELS > 0
    dImgPostProcess = dNUM_LEVELS * imadjust(dImgPostProcess ./ dNUM_LEVELS, [], [], objErrorParams.dGammaValue);
end
% Contrast adjustment TBC
% dImgPostProcess = imadjust(dImgPostProcess, ...
%                 stretchlim(dImgPostProcess, [0.1 0.95]), []);

% Datatype conversion
varImgPostProcess = cast(dImgPostProcess, objOutputSettings.charOutputDataType);

end

function dImgPostProcess = BloomBrightRowsCols_(dInputImg, ...
                                                dBloomProbRow, ...
                                                dBloomProbCol, ...
                                                dLambdaBloomRow, ...
                                                dLambdaBloomCol, ...
                                                dPartialBloomRowFraction, ...
                                                dPartialBloomColFraction, ...
                                                dNUM_LEVELS, ...
                                                bApplyBloomBlur)
%% SIGNATURE
% dImgPostProcess = BloomBrightRowsCols_(dInputImg, dBloomProbRow, dBloomProbCol, dLambdaBloomRow, ...
%     dLambdaBloomCol, dPartialBloomRowFraction, dPartialBloomColFraction, dNUM_LEVELS, bApplyBloomBlur)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
%   1) Find the bright pixels above 0.95 * dNUM_LEVELS in dInputImg.
%   2) For each bright pixel, draw whether row bloom and column bloom trigger.
%   3) If triggered, partially bloom that pixel's row or column, using
%      a Gaussian roll-off around the bright spot.
% Notes:
%   - Intensities only increase; the function never reduces them.
%   - Row and column bloom are independent: a bright pixel may cause
%     row bloom, column bloom, both, or neither (based on random draws).
%   - dBrightSpotThreshold, dMaxHalfWidthRow, dMaxHalfHeightCol and the roll-off
%     sigmas set how blooming appears.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
%   dInputImg                 - original image (double).
%   dBloomProbRow             - probability that a bright pixel triggers row bloom.
%   dBloomProbCol             - probability that a bright pixel triggers column bloom.
%   dLambdaBloomRow           - Poisson mean (# of row bloom events).
%   dLambdaBloomCol           - Poisson mean (# of column bloom events).
%   dPartialBloomRowFraction  - maximum row bloom half-width as a fraction of the columns.
%   dPartialBloomColFraction  - maximum column bloom half-height as a fraction of the rows.
%   dNUM_LEVELS               - maximum intensity (e.g., 255 for uint8).
%   bApplyBloomBlur           - blur the bloomed regions afterwards (default true).
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
%   dImgPostProcess  - image with row & column bloom effects added.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% AUTHORS: Pietro Califano, chatGPTo1
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Standard header; prefixes match the stored class.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dInputImg                   % Input image (double)
    dBloomProbRow               % Probability of row bloom
    dBloomProbCol               % Probability of column bloom
    dLambdaBloomRow             % Poisson param for row bloom
    dLambdaBloomCol             % Poisson param for column bloom
    dPartialBloomRowFraction
    dPartialBloomColFraction
    dNUM_LEVELS               % Max intensity (e.g., 255 for 8-bit)
    bApplyBloomBlur (1,1) logical = true
end

% Copy input image to output so we can modify in-place
dImgPostProcess = dInputImg;

% Image size
[dNumRows, dNumCols] = size(dImgPostProcess);

% 1) Choose a brightness threshold
dBrightSpotThreshold = 0.95 * dNUM_LEVELS;

% 2) Find all "seed" pixels that exceed the threshold
[dBrightRowIdx, dBrightColIdx] = find(dImgPostProcess > dBrightSpotThreshold);

% Parameters for row-based partial bloom
dMaxHalfWidthRow = max(1, round(dPartialBloomRowFraction * dNumCols));    % fraction of the columns

% Define a sigma for the Gaussian roll-off (row direction)
dSigmaBloomRowRollOff = 0.3 * dMaxHalfWidthRow;

% Parameters for column-based partial bloom
dMaxHalfHeightCol   = max(1, round(dPartialBloomColFraction * dNumRows));   % fraction of the rows
dSigmaBloomColRollOff = 0.3 * dMaxHalfHeightCol;

% 3) For each bright pixel, decide if row bloom triggers and/or col bloom triggers
dNumBrightSpots = numel(dBrightRowIdx);

for ui32SpotIdx = uint32(1):uint32(dNumBrightSpots)

    dRowCenter = dBrightRowIdx(ui32SpotIdx);
    dColCenter = dBrightColIdx(ui32SpotIdx);

    % ---- ROW BLOOM CHECK ----
    if rand < dBloomProbRow  % Probability threshold
        if poissrnd(dLambdaBloomRow) > 0

            % Row bloom triggered by this bright pixel
            dHalfWidth = randi([1, dMaxHalfWidthRow]);
            dStartCol = max(1, dColCenter - dHalfWidth);
            dEndCol   = min(dNumCols, dColCenter + dHalfWidth);

            % Build a coordinate array for the columns in [startCol : endCol]
            dColsRange = dStartCol : dEndCol;
            dDistanceFromCenter = double(dColsRange - dColCenter); % Negative to positive

            % 1D Gaussian roll-off in horizontal direction
            dScaleFactor = exp( - (dDistanceFromCenter.^2) / (2 * dSigmaBloomRowRollOff^2) );

            % Raise intensities in that partial row section
            for ui32ColRangeIdx = uint32(1):uint32(length(dColsRange))
                dColIdx = dColsRange(ui32ColRangeIdx);
                % dScaleFactor = 1 => saturate, dScaleFactor < 1 => less intense
                dNewVal = dNUM_LEVELS * dScaleFactor(ui32ColRangeIdx);
                dImgPostProcess(dRowCenter, dColIdx) = max(dImgPostProcess(dRowCenter, dColIdx), dNewVal);
            end
        end
    end

    % ---- COLUMN BLOOM CHECK ----
    if rand < dBloomProbCol
        if poissrnd(dLambdaBloomCol) > 0
            % Column bloom triggered by this bright pixel
            dHalfHeight = randi([1, dMaxHalfHeightCol]);
            dStartRow = max(1, dRowCenter - dHalfHeight);
            dEndRow   = min(dNumRows, dRowCenter + dHalfHeight);

            % Build a coordinate array for the rows in [startRow : endRow]
            dRowsRange = dStartRow : dEndRow;
            dDistanceFromCenter = double(dRowsRange - dRowCenter);
            % 1D Gaussian roll-off in vertical direction
            dScaleFactor = exp( - (dDistanceFromCenter.^2) / (2 * dSigmaBloomColRollOff^2) );

            % Raise intensities in that partial column section
            for ui32RowRangeIdx = uint32(1):uint32(length(dRowsRange))
                dRowIdx = dRowsRange(ui32RowRangeIdx);
                dNewVal = dNUM_LEVELS * dScaleFactor(ui32RowRangeIdx);
                dImgPostProcess(dRowIdx, dColCenter) = max(dImgPostProcess(dRowIdx, dColCenter), dNewVal);
            end
        end
    end
end

if bApplyBloomBlur
    [dImgPostProcess] = ApplyBlurToBlooming_(dImgPostProcess, dInputImg);
end

% Final clamp of intensity values
dImgPostProcess = max(0, min(dImgPostProcess, dNUM_LEVELS));


end

function [dBloomedImg] = ApplyBlurToBlooming_(dBloomedImg, dImgBeforeBloom)
%% SIGNATURE
% dBloomedImg = ApplyBlurToBlooming_(dBloomedImg, dImgBeforeBloom)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Blur the pixels that blooming brightened, and keep the brighter of the bloomed and blurred values.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dBloomedImg      Image after blooming (double, DN).
% dImgBeforeBloom  Same image before blooming.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dBloomedImg      Image with the bloomed regions blurred.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header.
% -------------------------------------------------------------------------------------------------------------

% Mark the pixels that blooming brightened by at least the tolerance
dBloomTolerance = 0.1;
bBloomMask = (dBloomedImg - dImgBeforeBloom) >= dBloomTolerance;

% Dilate blooming mask
objStrel = strel('disk', 3);
bBloomMask = imdilate(bBloomMask, objStrel);

% Copy the bloomed regions into an image that is 0 outside the mask
dBloomOnly = zeros(size(dBloomedImg));
dBloomOnly(bBloomMask) = dBloomedImg(bBloomMask);

% Apply a small Gaussian blur to the bloomed regions
dBloomBlurred = imgaussfilt(dBloomOnly, 0.1, "FilterSize", 7);

% Merge the blurred bloom back into the image

% Keep the brighter of the bloomed and blurred values inside the mask
dBloomedImg(bBloomMask) = max(dBloomedImg(bBloomMask), dBloomBlurred(bBloomMask));
end
