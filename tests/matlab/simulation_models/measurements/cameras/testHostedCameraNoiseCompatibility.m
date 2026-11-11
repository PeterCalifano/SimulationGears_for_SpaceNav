function objTests = testHostedCameraNoiseCompatibility()
%% SIGNATURE
% objTests = testHostedCameraNoiseCompatibility()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Exercise read-only native/hosted noise ownership without a renderer build.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-10-2026  Pietro Califano, Codex gpt-6.1  Cover preview noise ownership.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% AssertHostedCameraNoiseCompatible, CNativeDetectorRendererFixture.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testPreviewProviderRetainsCallerStream(testCase)
% A preview provider accepts hosted noise; the guard draws nothing and renders nothing.
strBefore = rng;
objSettings = SImageErrorsPipelineSettings('bAddShotNoise', true, ...
    'dSigmaDigitNumRelGenericWN', 5/255);
objRenderer = CCameraRendererFixture();

AssertHostedCameraNoiseCompatible(objSettings, objRenderer);

verifyEqual(testCase, rng, strBefore);
verifyEqual(testCase, objRenderer.ui32RenderCount, uint32(0));
end

function testNativeProviderRejectsHostedStochasticEffects(testCase)
% A native detector provider rejects each hosted stochastic effect on its own.
objRenderer = CNativeDetectorRendererFixture();
cellSettings = {SImageErrorsPipelineSettings('bAddShotNoise', true), ...
    SImageErrorsPipelineSettings('dSigmaDigitNumRelGenericWN', 0.01), ...
    SImageErrorsPipelineSettings('dDeadPixelPerc', 0.1), ...
    SImageErrorsPipelineSettings('dDeadBucketFraction', 0.1), ...
    SImageErrorsPipelineSettings('dBloomingProbability', 1, 'dBloomPoissonMean', 1), ...
    SImageErrorsPipelineSettings('dBrightSpotsProbability', 1, 'dBrightSpotsPoissonRate', 1)};
strBefore = rng;

for ui32Index = uint32(1):uint32(numel(cellSettings))
    verifyError(testCase, @() AssertHostedCameraNoiseCompatible( ...
        cellSettings{ui32Index}, objRenderer), ...
        'AssertHostedCameraNoiseCompatible:NativeDetectorProvider');
end

verifyEqual(testCase, rng, strBefore);
verifyEqual(testCase, objRenderer.ui32RenderCount, uint32(0));
end

function testNativeProviderAllowsCleanAndDeterministicEffects(testCase)
% A native detector provider accepts default settings and deterministic effects only.
objRenderer = CNativeDetectorRendererFixture();
objSettings = SImageErrorsPipelineSettings('dSigmaGenericBlur', 0.4, ...
    'dGammaValue', 2, 'dMeanGenericWN', 1, 'dBloomingProbability', 1, ...
    'dBrightSpotsPoissonRate', 1);
strBefore = rng;

AssertHostedCameraNoiseCompatible(SImageErrorsPipelineSettings(), objRenderer);
AssertHostedCameraNoiseCompatible(objSettings, objRenderer);

verifyEqual(testCase, rng, strBefore);
verifyEqual(testCase, objRenderer.ui32RenderCount, uint32(0));
end

function testRenderBoundaryRejectsBeforeAcquisition(testCase)
% RenderCameraFrame runs the guard before it moves the light or renders.
objRenderer = CNativeDetectorRendererFixture();
objSettings = SImageErrorsPipelineSettings('bAddShotNoise', true);
objRequest = SImageRequest(uint32(1), int32(0), 0, ...
    [10; 0; 0], [0; 0; 10], eye(3), zeros(3, 1), eye(3), ...
    'objImageErrorSettings', objSettings);
strBefore = rng;

verifyError(testCase, @() RenderCameraFrame(objRequest, objRenderer), ...
    'AssertHostedCameraNoiseCompatible:NativeDetectorProvider');

verifyEqual(testCase, rng, strBefore);
verifyEqual(testCase, objRenderer.ui32RenderCount, uint32(0));
verifyEqual(testCase, objRenderer.dSunPosition, zeros(3, 1));
end
