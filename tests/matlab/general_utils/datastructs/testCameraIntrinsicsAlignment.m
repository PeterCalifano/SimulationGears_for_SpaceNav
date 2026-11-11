function objTests = testCameraIntrinsicsAlignment()
%% SIGNATURE
% objTests = testCameraIntrinsicsAlignment()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify MATLAB [H,W] image order, [fx,fy] focal lengths and metadata copying.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-10-2026  Pietro Califano, Codex GPT-6  Check FoV, modern K and metadata preservation.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CCameraIntrinsics, SCameraSensorMetadata.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testFoVAndModernKUseTheirDeclaredAxisOrders(testCase)
% Use different focal lengths per axis so a swapped FoV formula shows
dImageSizeHW_px = [480, 800];
dFocalLength_uv_px = [1200, 800];
dPrincipalPoint_uv_px = [(dImageSizeHW_px(2) + 1) / 2, ...
                         (dImageSizeHW_px(1) + 1) / 2];
objCamera = CCameraIntrinsics(dFocalLength_uv_px, dPrincipalPoint_uv_px, ...
    dImageSizeHW_px, uint32(3));

dExpectedFovHW_rad = [2 * atan(dImageSizeHW_px(1) / (2 * dFocalLength_uv_px(2))); ...
                      2 * atan(dImageSizeHW_px(2) / (2 * dFocalLength_uv_px(1)))];
dExpectedFovHW_deg = rad2deg(dExpectedFovHW_rad);

verifyEqual(testCase, objCamera.dFovHW, dExpectedFovHW_rad, 'AbsTol', 1e-12);
verifyEqual(testCase, objCamera.dMeanIFovInRad, ...
    dExpectedFovHW_rad ./ dImageSizeHW_px(:), 'AbsTol', 1e-12);
verifyEqual(testCase, objCamera.PrincipalPoint, dPrincipalPoint_uv_px);
verifyEqual(testCase, objCamera.K, [dFocalLength_uv_px(1), 0, dPrincipalPoint_uv_px(1); ...
                                   0, dFocalLength_uv_px(2), dPrincipalPoint_uv_px(2); ...
                                   0, 0, 1]);
verifyEqual(testCase, CCameraIntrinsics.computeFovInDegrees( ...
    dImageSizeHW_px(:), dFocalLength_uv_px(:)), dExpectedFovHW_deg', 'AbsTol', 1e-12);
verifyEqual(testCase, objCamera.ui32NumOfChannels, uint32(3));
end

function testCopyKeepsMetadataAndRequestedChannelCount(testCase)
% A copy keeps K, the image size, the requested channel count and the sensor metadata.
objCamera = CCameraIntrinsics([1200, 800], [400.5, 240.5], [480, 800], uint32(3));
objCamera.objSensorMetadata = SCameraSensorMetadata('dGainElectronsPerDN', 42);
objCameraCopy = CCameraIntrinsics.fromCameraIntrinsics(objCamera, uint32(3));

verifyEqual(testCase, objCameraCopy.K, objCamera.K);
verifyEqual(testCase, objCameraCopy.ImageSize, objCamera.ImageSize);
verifyEqual(testCase, objCameraCopy.ui32NumOfChannels, uint32(3));
verifyEqual(testCase, objCameraCopy.objSensorMetadata, objCamera.objSensorMetadata);
end
