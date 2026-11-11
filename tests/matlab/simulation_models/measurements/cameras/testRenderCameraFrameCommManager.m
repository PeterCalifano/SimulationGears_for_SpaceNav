function objTests = testRenderCameraFrameCommManager()
%% SIGNATURE
% objTests = testRenderCameraFrameCommManager()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Opt-in check of RenderCameraFrame against the real BlenderPyCommManager. RenderCameraFrame names
% the rendering frame as text; the real class must convert it to the same frame as its enum.
% Skipped unless SIM_GEARS_COMM_MANAGER_DIR put CommManager4MATLAB on the path.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the opt-in CommManager4MATLAB check.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RenderCameraFrame, SImageRequest, CBlenderCommManagerCaptureProbe, BlenderPyCommManager,
% EnumRenderingFrame.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Skip the suite unless CommManager4MATLAB is on the path.
bAvailable = ~isempty(meta.class.fromName('BlenderPyCommManager')) && ...
    ~isempty(meta.class.fromName('EnumRenderingFrame'));
assumeTrue(testCase, bAvailable, ['Needs CommManager4MATLAB (BlenderPyCommManager, EnumRenderingFrame). ', ...
    'Set SIM_GEARS_COMM_MANAGER_DIR to its root before SetupSimGears.']);
end

function testTextFrameComposesTheCustomFrameScene(testCase)
% The scene vector from the text frame name equals the one the class composes for the enum.
dRotation = [0, -1, 0; 1, 0, 0; 0, 0, 1];
dTarget = [10; 20; 30];
objRequest = SImageRequest(uint32(3), int32(2), 0.75, ...
    dTarget + [100; 0; 0], dTarget + [0; 0; 10], eye(3), dTarget, dRotation);
objRenderer = CBlenderCommManagerCaptureProbe();

dImage = RenderCameraFrame(objRequest, objRenderer);

dExpectedScene = BlenderPyCommManager.composeSceneDataVector(objRequest.dSunPosition_W, ...
    objRequest.dCameraPosition_W, ...
    objRequest.dDCM_WfromCAM, ...
    objRequest.dTargetPosition_W, ...
    objRequest.dDCM_WfromTB, ...
    'enumRenderingFrame', EnumRenderingFrame.CUSTOM_FRAME, ...
    'bConvertCamQuatToBlenderQuat', true);

verifyEqual(testCase, objRenderer.dSceneDataVector, dExpectedScene);
verifyEqual(testCase, objRenderer.ui32InitializeCount, uint32(1));
verifyEqual(testCase, dImage, repmat(128, 4, 5), 'AbsTol', 1e-12);
end
