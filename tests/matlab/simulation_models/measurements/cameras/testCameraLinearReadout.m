function objTests = testCameraLinearReadout()
%% SIGNATURE
% objTests = testCameraLinearReadout()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify the optional signed readout shares one render and one noise realization.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-10-2026  Pietro Califano, Codex GPT-6  Verify signed camera readout behavior.
% 19-10-2026  Pietro Califano, Codex GPT-6  Cover detector scale resolution.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass; typed loop counters.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RenderCameraFrame, CCameraBlenderRendererFixture, ApplyCameraImageErrors.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testCleanAcquisitionReturnsNormalizedSingleSignal(objTestCase)
% Default settings return the raw render and its single-precision signal over 255 DN.
objRequest = BuildRequest_(SImageErrorsPipelineSettings());
objRenderer = CCameraBlenderRendererFixture();

[dImage, strDiagnostics, dRawImage, ~, fSignal] = RenderCameraFrame(objRequest, objRenderer);

verifyEqual(objTestCase, objRenderer.ui32RenderCount, uint32(1));
verifyEqual(objTestCase, dRawImage, dImage);
verifyClass(objTestCase, fSignal, 'single');
verifyEqual(objTestCase, fSignal, single(dRawImage/255));
verifyTrue(objTestCase, strDiagnostics.bHasLinearDetectorImage);
verifyEqual(objTestCase, strDiagnostics.dReadNoiseSigmaNorm, 0);
verifyEqual(objTestCase, strDiagnostics.dDetectorNormalizationLevels, 255);
verifyFalse(objTestCase, isfield(strDiagnostics, 'fLinearDetectorImage'));
end

function testNoisyReadoutUsesSameRenderAndNoiseDraw(objTestCase)
% The rendered noisy image and signal match one ApplyCameraImageErrors call on the same seed.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
objSettings = SImageErrorsPipelineSettings('charOutputDataType', 'double', ...
    'bAddShotNoise', true, 'dGainElectronsPerDN', 32, ...
    'dSigmaDigitNumRelGenericWN', 3.5/255);
objRequest = BuildRequest_(objSettings);
objRenderer = CCameraBlenderRendererFixture();

rng(1409, 'twister');
[dImage, strDiagnostics, dRawImage, ~, fSignal] = RenderCameraFrame(objRequest, objRenderer);
strRenderRNG = rng;

rng(1409, 'twister');
[dExpected, fExpectedSignal, dExpectedSigma] = ApplyCameraImageErrors(dRawImage, objSettings);
strExpectedRNG = rng;

verifyEqual(objTestCase, objRenderer.ui32RenderCount, uint32(1));
verifyEqual(objTestCase, dImage, dExpected, 'AbsTol', 0);
verifyEqual(objTestCase, fSignal, fExpectedSignal, 'AbsTol', 0);
verifyEqual(objTestCase, strRenderRNG, strExpectedRNG);
verifyTrue(objTestCase, strDiagnostics.bHasLinearDetectorImage);
verifyEqual(objTestCase, strDiagnostics.dReadNoiseSigmaNorm, dExpectedSigma, 'AbsTol', 0);
verifyEqual(objTestCase, strDiagnostics.dDetectorNormalizationLevels, 255);
end

function testAutomaticCleanReadoutUsesIntegerAndFloatScales(objTestCase)
% Automatic levels follow the integer storage range, or 255 DN for float renders.
strBefore = rng;
objCleanup = onCleanup(@() rng(strBefore));
objSettings = SImageErrorsPipelineSettings('dNUM_LEVELS', 0);
verifyTrue(objTestCase, objSettings.bDefaultConstructed);

rng(8761, 'twister');
strExpectedRng = rng;
cellRenderedImages = {uint8(128*ones(4, 5)), ...
    uint16(128*ones(4, 5)), single(0.5*ones(4, 5))};
cellExpectedRawImages = {uint8(128*ones(4, 5)), ...
    uint16(128*ones(4, 5)), single(127.5*ones(4, 5))};
dExpectedLevels = [255, 65535, 255];

% Automatic clean scaling uses integer storage range, or the renderer's fixed 8-bit float DN.
for ui32CaseIdx = uint32(1):uint32(numel(cellRenderedImages))
    objRequest = BuildRequest_(objSettings);
    objRenderer = CCameraBlenderRendererFixture();
    objRenderer.dImage = cellRenderedImages{ui32CaseIdx};

    [varImage, strDiagnostics, varRawImage, ~, fSignal] = RenderCameraFrame(objRequest, objRenderer);

    verifyEqual(objTestCase, varRawImage, cellExpectedRawImages{ui32CaseIdx});
    verifyEqual(objTestCase, varImage, varRawImage);
    verifyEqual(objTestCase, strDiagnostics.dDetectorNormalizationLevels, ...
        dExpectedLevels(ui32CaseIdx));
    verifyEqual(objTestCase, fSignal, single(double(varRawImage)./dExpectedLevels(ui32CaseIdx)));
    verifyEqual(objTestCase, strDiagnostics.dReadNoiseSigmaNorm, 0);
    verifyTrue(objTestCase, strDiagnostics.bHasLinearDetectorImage);
    verifyEqual(objTestCase, objRenderer.ui32RenderCount, uint32(1));
    verifyEqual(objTestCase, rng, strExpectedRng);
end
end

function testCustomLevelsAreReportedByRenderDiagnostics(objTestCase)
% Explicit levels and read noise appear in the render diagnostics.
objSettings = SImageErrorsPipelineSettings('dNUM_LEVELS', 100, ...
    'dSigmaDigitNumRelGenericWN', 3.5/100);
objRequest = BuildRequest_(objSettings);
objRenderer = CCameraBlenderRendererFixture();

[~, strDiagnostics, ~, ~, fSignal] = RenderCameraFrame(objRequest, objRenderer);

verifyTrue(objTestCase, strDiagnostics.bHasLinearDetectorImage);
verifyEqual(objTestCase, strDiagnostics.dDetectorNormalizationLevels, 100);
verifyEqual(objTestCase, strDiagnostics.dReadNoiseSigmaNorm, 3.5/100);
verifyEqual(objTestCase, objRenderer.ui32RenderCount, uint32(1));
verifySize(objTestCase, fSignal, [4, 5]);
end

function testExampleReturnsTheDocumentedSignal(objTestCase)
% RunCameraFrameExample returns the 4-by-5 image and the signal its header documents.
objRenderer = CCameraRendererFixture();

[ui8Image, strDiagnostics, fSignal] = RunCameraFrameExample(objRenderer);

verifyEqual(objTestCase, objRenderer.ui32RenderCount, uint32(1));
verifyEqual(objTestCase, size(ui8Image), [4, 5]);
verifyTrue(objTestCase, strDiagnostics.bHasLinearDetectorImage);
verifyEqual(objTestCase, strDiagnostics.dDetectorNormalizationLevels, 255);
verifyEqual(objTestCase, fSignal, single(double(ui8Image)/255));
end

function testFourthOutputRemainsSurfaceLabels(objTestCase)
% Requesting the fifth output does not move the labels out of the fourth.
strLabelsExpected = struct('status', uint32(3*ones(4, 5)));
objRequest = BuildRequest_(SImageErrorsPipelineSettings());

objFourOutputRenderer = CCameraRendererFixture();
objFourOutputRenderer.strSurfaceLabels = strLabelsExpected;
[ui8FourOutput, ~, ~, strFourOutputLabels] = RenderCameraFrame( ...
    objRequest, objFourOutputRenderer, 'bCaptureSurfaceLabels', true);

objFiveOutputRenderer = CCameraRendererFixture();
objFiveOutputRenderer.strSurfaceLabels = strLabelsExpected;
[ui8FiveOutput, ~, ~, strFiveOutputLabels, fSignal] = RenderCameraFrame( ...
    objRequest, objFiveOutputRenderer, 'bCaptureSurfaceLabels', true);

verifyEqual(objTestCase, ui8FourOutput, ui8FiveOutput);
verifyEqual(objTestCase, strFourOutputLabels, strLabelsExpected);
verifyEqual(objTestCase, strFiveOutputLabels, strLabelsExpected);
verifyClass(objTestCase, fSignal, 'single');
verifyEqual(objTestCase, objFourOutputRenderer.ui32RenderCount, uint32(1));
verifyEqual(objTestCase, objFiveOutputRenderer.ui32RenderCount, uint32(1));
verifyEqual(objTestCase, objFiveOutputRenderer.strCaptureInfo.frame_id, '2');
end

function testDownstreamEffectsMarkLinearSignalUnavailable(objTestCase)
% Gamma, dead pixels or columns, bloom and bright spots each withhold the linear signal.
cellSettings = { ...
    SImageErrorsPipelineSettings('dGammaValue', 2), ...
    SImageErrorsPipelineSettings('dDeadPixelPerc', 0.2), ...
    SImageErrorsPipelineSettings('dDeadBucketFraction', 0.25), ...
    SImageErrorsPipelineSettings('dBloomingProbability', 1, 'dBloomPoissonMean', 2), ...
    SImageErrorsPipelineSettings('dBrightSpotsProbability', 1, 'dBrightSpotsPoissonRate', 1)};

for ui32CaseIdx = uint32(1):uint32(numel(cellSettings))
    objRequest = BuildRequest_(cellSettings{ui32CaseIdx});
    objRenderer = CCameraBlenderRendererFixture();

    [dImage, strDiagnostics, ~, ~, fSignal] = RenderCameraFrame(objRequest, objRenderer);

    verifyFalse(objTestCase, strDiagnostics.bHasLinearDetectorImage);
    verifyEmpty(objTestCase, fSignal);
    verifyEqual(objTestCase, objRenderer.ui32RenderCount, uint32(1));
    verifyEqual(objTestCase, size(dImage), [4, 5]);
end
end

function objRequest = BuildRequest_(objSettings)
% Build the shared acquisition: camera on +Z, Sun on +X, target at the origin.
arguments (Input)
    objSettings (1,1) SImageErrorsPipelineSettings
end

objRequest = SImageRequest(uint32(9), int32(2), 0.5, [10; 0; 0], [0; 0; 10], ...
    eye(3), zeros(3, 1), eye(3), 'objImageErrorSettings', objSettings);
end
