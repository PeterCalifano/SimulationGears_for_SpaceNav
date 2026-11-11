function objTests = testRenderCameraFrame()
%% SIGNATURE
% objTests = testRenderCameraFrame()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Exercise identity, complete world-to-target conversion and renderer failures.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-10-2026  Pietro Califano, Codex gpt-6.1  Add acquisition-boundary regressions.
% 19-10-2026  Pietro Califano, Codex GPT-6  Expect additive scalar signal diagnostics.
% 19-10-2026  Pietro Califano, Codex GPT-6  Verify detector normalization diagnostics.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header; readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SImageRequest, RenderCameraFrame, CCameraRendererFixture.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testTranslatedRotatedScene(testCase)
% Rotate the target 90 deg about Z and translate it; the renderer receives target-body geometry.
dRotation = [0, -1, 0; 1, 0, 0; 0, 0, 1];
dTarget = [10; 20; 30];
objRequest = SImageRequest(uint32(7), int32(3), 1.25, ...
    dTarget + [100; 0; 0], dTarget + [0; 0; 10], eye(3), dTarget, dRotation);
objRenderer = CCameraRendererFixture();

[ui8Image, strDiagnostics] = RenderCameraFrame(objRequest, objRenderer);

verifyEqual(testCase, objRenderer.dSunPosition, [0; -100; 0]);
verifyEqual(testCase, objRenderer.dCameraPosition, [0; 0; 10]);
verifyEqual(testCase, objRenderer.dCameraRotation, dRotation');
verifyEqual(testCase, objRenderer.ui32RenderCount, uint32(1));
verifyEqual(testCase, objRenderer.ui32TimestampID, uint32(7));
verifyEqual(testCase, ui8Image, repmat(uint8(128), 4, 5));
verifyEqual(testCase, strDiagnostics, struct('ui32TimestampID', uint32(7), ...
    'i32FrameID', int32(3), ...
    'dTimestamp', 1.25, ...
    'bHasLinearDetectorImage', true, ...
    'dReadNoiseSigmaNorm', 0, ...
    'dDetectorNormalizationLevels', 255));
end

function testTargetCenteredLegacyParity(testCase)
% With the target at the origin and identity attitude, world and target-body geometry agree.
objRequest = SImageRequest(uint32(1), int32(0), 0, [10; 0; 0], [0; 0; 10], eye(3), zeros(3, 1), eye(3));
objRenderer = CCameraRendererFixture();

RenderCameraFrame(objRequest, objRenderer);

verifyEqual(testCase, objRenderer.dCameraPosition, [0; 0; 10]);
verifyEqual(testCase, objRenderer.dSunPosition, [10; 0; 0]);
end

function testPackedDoubleWrapperParity(testCase)
% Spectra's existing Eigen/gtwrap interface returns packed uint32 words as doubles.
objRequest = SImageRequest(uint32(1), int32(0), 0, [10; 0; 0], [0; 0; 10], eye(3), zeros(3, 1), eye(3));
objRenderer = CCameraRendererFixture();
ui8ExpectedImage = RenderCameraFrame(objRequest, objRenderer);

objRenderer.ui32Image = double(objRenderer.ui32Image);
ui8ActualImage = RenderCameraFrame(objRequest, objRenderer);
verifyEqual(testCase, ui8ActualImage, ui8ExpectedImage);

% Reject negative, fractional, infinite and out-of-range words
for dInvalidWord = [-1, 0.5, Inf, 2^32]
    objRenderer.ui32Image(1) = dInvalidWord;
    verifyError(testCase, @() RenderCameraFrame(objRequest, objRenderer), ...
        'RenderCameraFrame:InvalidRendererImage');
end
end

function testInvalidRotationAndGeometry(testCase)
% Reject an improper camera rotation and a camera placed at the target origin.
verifyError(testCase, @() SImageRequest(uint32(1), int32(0), 0, ...
    [10; 0; 0], [0; 0; 10], diag([1, 1, -1]), zeros(3, 1), eye(3)), ...
    'SImageRequest:InvalidRotation');
verifyError(testCase, @() SImageRequest(uint32(1), int32(0), 0, ...
    [10; 0; 0], zeros(3, 1), eye(3), zeros(3, 1), eye(3)), ...
    'SImageRequest:DegenerateGeometry');
end

function testInvalidRendererAndEmptyPixels(testCase)
% Reject a provider without a renderer protocol and an empty packed image.
objRequest = SImageRequest(uint32(1), int32(0), 0, [10; 0; 0], [0; 0; 10], eye(3), zeros(3, 1), eye(3));
verifyError(testCase, @() RenderCameraFrame(objRequest, struct()), 'RenderCameraFrame:UnsupportedRenderer');

objRenderer = CCameraRendererFixture();
objRenderer.ui32Image = uint32([]);
verifyError(testCase, @() RenderCameraFrame(objRequest, objRenderer), 'RenderCameraFrame:InvalidRendererImage');
end
