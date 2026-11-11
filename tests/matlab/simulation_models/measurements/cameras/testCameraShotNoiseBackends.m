function objTests = testCameraShotNoiseBackends()
%% SIGNATURE
% objTests = testCameraShotNoiseBackends()
% -------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify explicit shot-sampler selection, caller-owned randomness and unchanged camera
% effect order. Accelerated cases require their independently built MEX modules; run
% the suite with both modules on the path to qualify an accelerated installation.
% -------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests    MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-10-2026  Pietro Califano     Cover reusable MATLAB, CPU MEX and CUDA shot sampling.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SampleCameraShotCharge, ApplyCameraImageErrors, SImageErrorsPipelineSettings.
% -------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function setup(objTestCase)
% Restore the caller RNG state after each test.
strOriginalRng = rng;
objTestCase.addTeardown(@() rng(strOriginalRng));
end

function testMatlabBackendPreservesOriginalPoissonSequence(objTestCase)
% The matlab backend draws the same poissrnd sequence and returns no exposure key.
dExpectations = reshape(linspace(0, 6300, 120), 10, 12);
rng(731, 'twister');
dExpected = poissrnd(dExpectations);
strExpectedRng = rng;
rng(731, 'twister');
[dActual, ui64ExposureSeed] = SampleCameraShotCharge(dExpectations, "matlab");
verifyEqual(objTestCase, dActual, dExpected);
verifyEqual(objTestCase, rng, strExpectedRng);
verifyEqual(objTestCase, ui64ExposureSeed, uint64(0));
end

function testDisabledShotEffectConsumesNoBackendRandomness(objTestCase)
% With shot noise off, no backend changes the image or the RNG state, even without its MEX.
ui8Input = uint8(reshape(0:119, 10, 12));
for charBackend = ["matlab", "cpu_mex", "cuda"]
    objSettings = SImageErrorsPipelineSettings('charShotNoiseBackend', charBackend);
    strExpectedRng = rng;
    [ui8Actual, fSignedSignal] = ApplyCameraImageErrors(ui8Input, objSettings);
    verifyEqual(objTestCase, ui8Actual, ui8Input);
    verifyEqual(objTestCase, fSignedSignal, single(double(ui8Input)/255));
    verifyEqual(objTestCase, rng, strExpectedRng);
end
end

function testCpuMexRepeatsAfterCallerRngReset(objTestCase)
% Resetting the caller RNG repeats the cpu_mex exposure and the following RNG state.
assumeEqual(objTestCase, exist('DrawPoissonCountsCpu_mex', 'file'), 3, ...
    'Build the maintained CPU fallback before qualifying this backend.');
VerifyBackendReplay_(objTestCase, "cpu_mex");
end

function testCudaRepeatsAfterCallerRngReset(objTestCase)
% Resetting the caller RNG repeats the cuda exposure and the following RNG state.
RequireCudaMex_(objTestCase);
VerifyBackendReplay_(objTestCase, "cuda");
end

function testCudaKeyUsesOnlyCallerRandomState(objTestCase)
% The cuda exposure key comes from two uint32 draws of the caller RNG and nothing else.
RequireCudaMex_(objTestCase);
dExpectations = 100*ones(17, 19);
rng(1905, 'twister');
ui32SeedWords = randi([0, double(intmax('uint32'))], 1, 2, 'uint32');
ui64ExpectedSeed = bitor(bitshift(uint64(ui32SeedWords(1)), 32), uint64(ui32SeedWords(2)));
strExpectedRng = rng;
dExpected = DrawPoissonCountsCuda_mex(dExpectations, ui64ExpectedSeed, uint32(0));
verifyEqual(objTestCase, rng, strExpectedRng);
rng(1905, 'twister');
[dActual, ui64ActualSeed] = SampleCameraShotCharge(dExpectations, "cuda");
verifyEqual(objTestCase, dActual, dExpected);
verifyEqual(objTestCase, ui64ActualSeed, ui64ExpectedSeed);
verifyEqual(objTestCase, rng, strExpectedRng);
end

function testCudaChunkingAndCallOrderPreservePixelIdentity(objTestCase)
% Splitting the image into chunks with a pixel offset gives the same draw per pixel.
RequireCudaMex_(objTestCase);
dExpectations = reshape(linspace(0, 12000, 4096), 64, 64);
ui64Seed = bitor(bitshift(uint64(1), 61), uint64(20261019));
dReference = DrawPoissonCountsCuda_mex(dExpectations, ui64Seed, uint32(17));
dOther = DrawPoissonCountsCuda_mex(dExpectations, ui64Seed, uint32(18));
verifyNotEqual(objTestCase, dReference, dOther);
dChunks = zeros(size(dExpectations));
dChunks(1:137) = DrawPoissonCountsCuda_mex(dExpectations(1:137), ui64Seed, uint32(17));
dChunks(138:end) = DrawPoissonCountsCuda_mex(dExpectations(138:end), ...
    ui64Seed, uint32(17), uint32(137));
verifyEqual(objTestCase, dChunks, dReference);
end

function testCudaRetainsBlurShotReadNoiseOrderAndSignedReadout(objTestCase)
% The cuda backend keeps the blur, shot, read-noise order and the unclipped signed signal.
RequireCudaMex_(objTestCase);
dInput = reshape(linspace(0, 255, 2048), 32, 64);
dGain = 6300/255;
objSettings = SImageErrorsPipelineSettings('charShotNoiseBackend', "cuda", ...
                                           'charOutputDataType', "double", ...
                                           'bAddShotNoise', true, ...
                                           'dSigmaGenericBlur', 0.4, ...
                                           'dGenericBlurFilterSize', int32(3), ...
                                           'dGainElectronsPerDN', dGain, ...
                                           'dSigmaDigitNumRelGenericWN', 3.5/255);

% Reproduce the public ordering without clipping shot excursions before read noise.
rng(918, 'twister');
dBlurred = imgaussfilt(dInput, 0.4, 'FilterSize', int32(3));
dShot = SampleCameraShotCharge(dBlurred*dGain, "cuda")/dGain;
dCombined = dShot + 3.5*randn(size(dInput));
strExpectedRng = rng;
rng(918, 'twister');
[dActual, fSignedSignal, dReadSigma, dLevels] = ApplyCameraImageErrors(dInput, objSettings);
verifyEqual(objTestCase, dActual, max(0, min(dCombined, 255)), 'AbsTol', 0);
verifyEqual(objTestCase, fSignedSignal, single(dCombined/255), 'AbsTol', 0);
verifyEqual(objTestCase, dReadSigma, 3.5/255);
verifyEqual(objTestCase, dLevels, 255);
verifyEqual(objTestCase, rng, strExpectedRng);
verifyTrue(objTestCase, any(fSignedSignal < 0 | fSignedSignal > 1, 'all'));
end

function VerifyBackendReplay_(objTestCase, charBackend)
% Preserve both the exposure and the caller's subsequent stream on a fixed backend.
dExpectations = reshape(linspace(0, 6300, 2048), 32, 64);
rng(8417, 'twister');
[dFirst, ui64FirstSeed] = SampleCameraShotCharge(dExpectations, charBackend);
strFirstRng = rng;
rng(8417, 'twister');
[dSecond, ui64SecondSeed] = SampleCameraShotCharge(dExpectations, charBackend);
verifyEqual(objTestCase, dFirst, dSecond);
verifyEqual(objTestCase, ui64FirstSeed, ui64SecondSeed);
verifyEqual(objTestCase, rng, strFirstRng);
verifyTrue(objTestCase, all(dFirst >= 0 & dFirst == floor(dFirst), 'all'));
end

function RequireCudaMex_(objTestCase)
% Skip the test when the cuda MEX is not on the path.
assumeEqual(objTestCase, exist('DrawPoissonCountsCuda_mex', 'file'), 3, ...
    'Build the standalone Spectra image_noise component before qualifying CUDA.');
end
