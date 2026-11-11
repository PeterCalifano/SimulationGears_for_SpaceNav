function objTests = testCameraImageErrors()
%% SIGNATURE
% objTests = testCameraImageErrors()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify charge/DN statistics, effect order, clipping, conversion and caller RNG.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-10-2026  Pietro Califano, Codex gpt-6.1  Cover migrated sensor-model defects.
% 18-10-2026  Pietro Califano, Codex gpt-6.1  Cover gain, independent noise and clipping.
% 19-10-2026  Pietro Califano, Codex GPT-6  Verify resolved detector normalization levels.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ApplyCameraImageErrors, SImageErrorsPipelineSettings, Image Processing and Statistics Toolboxes.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testDisabledEffectsPreservePixelsAndStream(testCase)
% With every effect off, the image passes through and the RNG state is untouched.
strBefore = rng;
ui8Input = uint8(reshape(1:20, 4, 5));
verifyEqual(testCase, ApplyCameraImageErrors(ui8Input), ui8Input);
verifyEqual(testCase, rng, strBefore);
[ui8Image, fSignal, dSigmaNorm, dNormalizationLevels] = ApplyCameraImageErrors(ui8Input);
verifyEqual(testCase, ui8Image, ui8Input);
verifyEqual(testCase, fSignal, single(double(ui8Input)/255));
verifyEqual(testCase, dSigmaNorm, 0);
verifyEqual(testCase, dNormalizationLevels, 255);
verifyEqual(testCase, rng, strBefore);
end

function testGammaCorrection(testCase)
% Gamma 2 maps mid-grey 128 to 64.
objSettings = SImageErrorsPipelineSettings('dGammaValue', 2);
ui8Image = ApplyCameraImageErrors(uint8(128*ones(4)), objSettings);
verifyEqual(testCase, ui8Image, uint8(64*ones(4)));
end

function testDeadColumnsDoNotEraseWholeFrame(testCase)
% A dead-column fraction of 0.25 zeroes 2 of 8 columns and leaves the rest.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
rng(71);
objSettings = SImageErrorsPipelineSettings('dDeadBucketFraction', 0.25);
ui8Image = ApplyCameraImageErrors(uint8(128*ones(4, 8)), objSettings);
verifyEqual(testCase, sum(all(ui8Image == 0, 1)), 2);
verifyEqual(testCase, sum(all(ui8Image == 128, 1)), 6);
end

function testSeededNoiseRepeatability(testCase)
% The same caller seed gives the same noisy image.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
objSettings = SImageErrorsPipelineSettings('bAddShotNoise', true, ...
                                           'dSigmaDigitNumRelGenericWN', 0.02);
rng(51);
ui8First = ApplyCameraImageErrors(uint8(128*ones(8)), objSettings);
rng(51);
ui8Second = ApplyCameraImageErrors(uint8(128*ones(8)), objSettings);
verifyEqual(testCase, ui8First, ui8Second);
end

function testGainRequiresPositiveFiniteRealScalar(testCase)
% Zero, negative, infinite, NaN and complex gains are rejected.
for dInvalidGain = [0, -1, Inf, NaN]
    if isfinite(dInvalidGain)
        charDiagnostic = 'MATLAB:validators:mustBePositive';
    else
        charDiagnostic = 'MATLAB:validators:mustBeFinite';
    end
    verifyError(testCase, @() SImageErrorsPipelineSettings( ...
        'dGainElectronsPerDN', dInvalidGain), charDiagnostic);
end
verifyError(testCase, @() SImageErrorsPipelineSettings('dGainElectronsPerDN', 1i), ...
    'MATLAB:validators:mustBeReal');
end

function testBlurChargeReadNoiseOrderAndStream(testCase)
% Blur, charge and read noise run in order, and the RNG ends in the same state for 1, 3 and 4 outputs.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
dGain = 6300/255;
dInput = reshape(linspace(20, 230, 90), 9, 10);
objSettings = SImageErrorsPipelineSettings('charOutputDataType', 'double', ...
                                           'dSigmaGenericBlur', 0.4, ...
                                           'dGenericBlurFilterSize', int32(3), ...
                                           'bAddShotNoise', true, ...
                                           'dGainElectronsPerDN', dGain, ...
                                           'dSigmaDigitNumRelGenericWN', 5/255);

% The public model contract fixes spatial blur before charge and independent read noise.
rng(319, 'twister');
dBlurredDN = imgaussfilt(dInput, 0.4, 'FilterSize', int32(3));
dChargeDN = poissrnd(dBlurredDN*dGain)/dGain;
dReadNoiseDN = 5*randn(size(dInput));
fExpectedSignal = single((dChargeDN + dReadNoiseDN) / 255);
dExpected = max(0, min(dChargeDN + dReadNoiseDN, 255));
strExpected = rng;

% Call with one, three and four outputs from the same seed
rng(319, 'twister');
dSingleOutput = ApplyCameraImageErrors(dInput, objSettings);
strSingleOutput = rng;
rng(319, 'twister');
[dActual, fLinearSignal, dSigmaNorm] = ApplyCameraImageErrors(dInput, objSettings);
strMultipleOutputs = rng;
rng(319, 'twister');
[dActualFour, fLinearSignalFour, dSigmaNormFour, dNormalizationLevels] = ...
    ApplyCameraImageErrors(dInput, objSettings);
strFourOutputs = rng;

verifyEqual(testCase, dActual, dExpected, 'AbsTol', 1e-12);
verifyEqual(testCase, dActualFour, dExpected, 'AbsTol', 1e-12);
verifyEqual(testCase, strSingleOutput, strExpected);
verifyEqual(testCase, strMultipleOutputs, strExpected);
verifyEqual(testCase, strFourOutputs, strExpected);
verifyEqual(testCase, dSingleOutput, dActual, 'AbsTol', 0);
verifyEqual(testCase, fLinearSignal, fExpectedSignal, 'AbsTol', 0);
verifyEqual(testCase, fLinearSignalFour, fExpectedSignal, 'AbsTol', 0);
verifyEqual(testCase, dSigmaNorm, 5/255, 'AbsTol', 0);
verifyEqual(testCase, dSigmaNormFour, 5/255, 'AbsTol', 0);
verifyEqual(testCase, dNormalizationLevels, 255);
verifyTrue(testCase, any(dActual ~= round(dActual), 'all'));
end

function testReportedNormalizationLevelsMatchSelectedOutputType(testCase)
% The reported levels and signed signal follow dNUM_LEVELS, not the output type.

% Custom levels with uint8 output
ui8Input = uint8([0, 25; 50, 100]);
objSettings100 = SImageErrorsPipelineSettings('charOutputDataType', 'uint8', ...
                                              'dNUM_LEVELS', 100);
[ui8Preview100, fSignal100, dSigma100, dLevels100] = ...
    ApplyCameraImageErrors(ui8Input, objSettings100);
verifyClass(testCase, ui8Preview100, 'uint8');
verifyEqual(testCase, dLevels100, 100);
verifyEqual(testCase, fSignal100, single(double(ui8Input)/100));
verifyEqual(testCase, dSigma100, 0);

% 8-bit levels with uint16 output
objSettings16 = SImageErrorsPipelineSettings('charOutputDataType', 'uint16', ...
                                             'dNUM_LEVELS', 255);
[ui16Preview16, fSignal16, dSigma16, dLevels16] = ...
    ApplyCameraImageErrors(ui8Input, objSettings16);
verifyClass(testCase, ui16Preview16, 'uint16');
verifyEqual(testCase, double(ui16Preview16), double(ui8Input));
verifyEqual(testCase, dLevels16, 255);
verifyEqual(testCase, fSignal16, single(double(ui8Input)/255));
verifyEqual(testCase, dSigma16, 0);
end

function testSignedDarkReadNoisePrecedesPreviewClipping(testCase)
% On a dark frame the signed signal keeps zero-mean read noise; the clipped preview has the half-normal mean.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
objSettings = SImageErrorsPipelineSettings('charOutputDataType', 'double', ...
                                           'dSigmaDigitNumRelGenericWN', 3.5/255);
rng(1801, 'twister');
[dPreview, fSignal, dSigma] = ApplyCameraImageErrors(zeros(256), objSettings);
dSignalDN = double(fSignal)*255;

verifyClass(testCase, fSignal, 'single');
verifyEqual(testCase, dSigma, 3.5/255, 'AbsTol', 0);
verifyLessThan(testCase, abs(mean(dSignalDN, 'all')), 0.08);
verifyEqual(testCase, std(dSignalDN(:)), 3.5, 'AbsTol', 0.04);
verifyEqual(testCase, mean(dPreview, 'all'), 3.5/sqrt(2*pi), 'AbsTol', 0.06);
end

function testLegacyNoiseAliasesReportResolvedSigma(testCase)
% Both legacy white-noise aliases resolve to the same normalized read-noise sigma.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
cellSettings = { ...
    SImageErrorsPipelineSettings('dSigmaGenericWNRel', 3.5/255), ...
    SImageErrorsPipelineSettings('dSigmaGenericWN', (3.5^2)/255)};
for ui32CaseIdx = uint32(1):uint32(numel(cellSettings))
    [~, ~, dSigma] = ApplyCameraImageErrors(zeros(4), cellSettings{ui32CaseIdx});
    verifyEqual(testCase, dSigma, 3.5/255, 'AbsTol', 1e-15);
end
end

function testChargeGainSetsExpectedVariance(testCase)
% The image variance is the shot variance I/g plus the read-noise variance.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
dGain = 6300/255;
objSettings = SImageErrorsPipelineSettings('charOutputDataType', 'double', ...
                                           'bAddShotNoise', true, ...
                                           'dGainElectronsPerDN', dGain, ...
                                           'dSigmaDigitNumRelGenericWN', 5/255);
rng(1607, 'twister');
dImage = ApplyCameraImageErrors(128*ones(256), objSettings);

% Independent shot and read noise add variances: I/g + sigma_read^2.
verifyEqual(testCase, mean(dImage, 'all'), 128, 'AbsTol', 0.12);
verifyEqual(testCase, var(dImage(:)), 128/dGain + 25, 'AbsTol', 0.7);
end

function testSaturationOccursAfterReadNoise(testCase)
% Clipping happens once, after read noise, not after the shot draw.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
objSettings = SImageErrorsPipelineSettings('charOutputDataType', 'double', ...
                                           'bAddShotNoise', true, ...
                                           'dGainElectronsPerDN', 1, ...
                                           'dSigmaDigitNumRelGenericWN', 20/255);

% Build the expected image with one clip after read noise, and the wrong early-clip variant
rng(704, 'twister');
dChargeDN = poissrnd(255*ones(20));
dReadNoise = 20*randn(20);
dExpected = max(0, min(dChargeDN + dReadNoise, 255));
dPrematureClip = max(0, min(min(dChargeDN, 255) + dReadNoise, 255));
rng(704, 'twister');
dActual = ApplyCameraImageErrors(255*ones(20), objSettings);

verifyEqual(testCase, dActual, dExpected, 'AbsTol', 1e-12);
verifyTrue(testCase, any(abs(dActual - dPrematureClip) > 1, 'all'));
verifyGreaterThanOrEqual(testCase, min(dActual, [], 'all'), 0);
verifyLessThanOrEqual(testCase, max(dActual, [], 'all'), 255);
end

function testFinalIntegerConversionMatchesFloatingDN(testCase)
% The uint8 output equals the double output cast to uint8 for the same seed.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
objSettings = SImageErrorsPipelineSettings('bAddShotNoise', true, ...
                                           'dGainElectronsPerDN', 8, ...
                                           'dSigmaDigitNumRelGenericWN', 5/255);
objFloatSettings = objSettings;
objFloatSettings.charOutputDataType = 'double';
ui8Input = uint8(repmat([0, 2, 128, 253, 255], 9, 1));
rng(509, 'twister');
dFloat = ApplyCameraImageErrors(ui8Input, objFloatSettings);
rng(509, 'twister');
ui8Image = ApplyCameraImageErrors(ui8Input, objSettings);

verifyClass(testCase, ui8Image, 'uint8');
verifyEqual(testCase, ui8Image, uint8(dFloat));
end

function testBloomHelpersExecuteForSmallImages(testCase)
% Blob, row and column blooming run on a 3x3 saturated image and keep its size.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
for charMode = ["blob", "row", "col"]
    rng(9);
    objSettings = SImageErrorsPipelineSettings('charBloomType', charMode, ...
                                               'dBloomingProbability', 1, ...
                                               'dBloomPoissonMean', 10, ...
                                               'dPartialBloomFraction', 0.01);
    ui8Image = ApplyCameraImageErrors(uint8(255*ones(3)), objSettings);
    verifyEqual(testCase, size(ui8Image), [3, 3]);
    verifyTrue(testCase, all(isfinite(ui8Image), 'all'));
end
end
