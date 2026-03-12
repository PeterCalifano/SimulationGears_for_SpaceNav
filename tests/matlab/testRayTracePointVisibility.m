function tests = testRayTracePointVisibility
tests = functiontests(localfunctions);
end

%%
function setupOnce(testCase)
% Test setup function
charThisScriptPath = fileparts(mfilename('fullpath'));
testCase.TestData.charThisScriptPath = charThisScriptPath;

charRootPath = fullfile(charThisScriptPath, "..", "..");
testCase.applyFixture( matlab.unittest.fixtures.PathFixture(charRootPath, IncludingSubfolders=true) );

charPrevDir = pwd;
cd(charThisScriptPath);
testCase.addTeardown(@cd, charPrevDir);

%%% Itokawa models
charShapeModelPath = fullfile(getenv('HOME'), "devDir", "rendering-sw", ...
                            "data", "asteroids", "Itokawa", "Itokawa_Hayabusa_50k_poly.obj");

testCase.assumeTrue(isfile(charShapeModelPath), ...
    "Shape model file is missing, skipping ray trace visibility tests.");

%%% Load shape model and target emulator
objShapeModel = CShapeModel("file_obj", charShapeModelPath, "km", "m", true);

ui32MaxNumPoints = 10000;
objTargetEmulator = CTargetEmulator(objShapeModel, ui32MaxNumPoints);

[dPosVector_W, dRot3_WfromTB] = objTargetEmulator.GetPose();

% Assign dummy but realistic data for test
dCameraPosition_TB      = [1000; 0; 0];
dSunPositionScaled_TB   = [980; 600; 0];
dSunPosition_TB         = 1e6 * dSunPositionScaled_TB;

ui32VerticesIDs         = uint32(objTargetEmulator.i32LandmarksID);
dPointsPositionsGT_TB   = objTargetEmulator.dPointsPositionsGT_TB(:,1:ui32MaxNumPoints);

% dDCM_INfromCAM (NOTE: IN is used here for legacy reasons)
objAttitudePointGeneratior = CAttitudePointingGenerator(dCameraPosition_TB, ...
                                                        dPosVector_W, ...
                                                        dSunPosition_TB);

[objAttitudePointGeneratior, dDCM_TBfromCAM] = objAttitudePointGeneratior.pointToTarget("dAuxiliaryAxis", [1;0;0]);

strShapeModel = objShapeModel.getShapeStruct();
strShapeModel = orderfields(strShapeModel);

strTargetBodyData.strShapeModel = strShapeModel;
strTargetBodyData.dDCM_INfromTB = dRot3_WfromTB;

strTargetBodyData = orderfields(strTargetBodyData);

% Camera (object and struct)
dFocalLength            = [5500, 5500];
ui32ImageSize           = [2048, 1536];
ui32OpticalCentre_uv    = 0.5 * ui32ImageSize;
objCameraIntrinsics = CCameraIntrinsics(dFocalLength, ui32OpticalCentre_uv, ui32ImageSize);  

% Define camera struct
strCameraData.dDCM_INfromCAM    = dDCM_TBfromCAM;
strCameraData.dPosition_IN      = dCameraPosition_TB; % TO MODIFY
strCameraData.dResX             = ui32ImageSize(1);
strCameraData.dResY             = ui32ImageSize(2);
strCameraData.dKcam             = objCameraIntrinsics.K;
strCameraData                   = orderfields(strCameraData);

% Assign data for CheckLMvisibility_rayTrace_MEX (LEGACY)
dFovHW = objCameraIntrinsics.GetFovHW;

strCamera.dFovX             = dFovHW(2); % [rad]
strCamera.dFovY             = dFovHW(1); % [rad]
strCamera.bIS_JPL_QUAT      = true;
strCamera.dCAMpos_IN        = dCameraPosition_TB;
strCamera = orderfields(strCamera);

% Determines max angle between Sun direction and LM direction --> for LOCAL PHASE ANGLE CHECK
strVisibilityCheckOptions.dIllumAngleThr = deg2rad(90); % [rad] % Lower value, more permissible
% Determines max angle between -los and LM direction in TF. Angle to assign is between +Los and the
strVisibilityCheckOptions.dLosAngleThr = deg2rad(10); % [rad] % Lower value, more permissible
strVisibilityCheckOptions = orderfields(strVisibilityCheckOptions);

strFcnOptions.dIllumAngleThr                = strVisibilityCheckOptions.dIllumAngleThr;
strFcnOptions.dLosAngleThr                  = strVisibilityCheckOptions.dLosAngleThr;
strFcnOptions.bENABLE_HEUR_GEOM_FEAS_CHECK  = true;
strFcnOptions.bTwoSidedTest = true;
strFcnOptions.bPointsAreMeshVertices = true;

strTargetBody = objTargetEmulator.getTargetStruct();

charRootDirRadiometricRGB_RT = getenv('WS_RENDER');

if not(isempty(charRootDirRadiometricRGB_RT))
    charRootDirRadiometricRGB_RT = fullfile(charRootDirRadiometricRGB_RT, "radiometric-rgb-raytracer");
else
    charRootDirRadiometricRGB_RT = fullfile(getenv('HOME'), "devDir", "rendering-sw", "radiometric-rgb-raytracer");
end

testCase.TestData.charShapeModelPath = charShapeModelPath;
testCase.TestData.objShapeModel = objShapeModel;
testCase.TestData.ui32MaxNumPoints = ui32MaxNumPoints;
testCase.TestData.objTargetEmulator = objTargetEmulator;
testCase.TestData.dPosVector_W = dPosVector_W;
testCase.TestData.dRot3_WfromTB = dRot3_WfromTB;
testCase.TestData.dCameraPosition_TB = dCameraPosition_TB;
testCase.TestData.dSunPositionScaled_TB = dSunPositionScaled_TB;
testCase.TestData.dSunPosition_TB = dSunPosition_TB;
testCase.TestData.ui32VerticesIDs = ui32VerticesIDs;
testCase.TestData.dPointsPositionsGT_TB = dPointsPositionsGT_TB;
testCase.TestData.objAttitudePointGeneratior = objAttitudePointGeneratior;
testCase.TestData.dDCM_TBfromCAM = dDCM_TBfromCAM;
testCase.TestData.strShapeModel = strShapeModel;
testCase.TestData.strTargetBodyData = strTargetBodyData;
testCase.TestData.dFocalLength = dFocalLength;
testCase.TestData.ui32ImageSize = ui32ImageSize;
testCase.TestData.ui32OpticalCentre_uv = ui32OpticalCentre_uv;
testCase.TestData.objCameraIntrinsics = objCameraIntrinsics;
testCase.TestData.strCameraData = strCameraData;
testCase.TestData.dFovHW = dFovHW;
testCase.TestData.strCamera = strCamera;
testCase.TestData.strVisibilityCheckOptions = strVisibilityCheckOptions;
testCase.TestData.strFcnOptions = strFcnOptions;
testCase.TestData.strTargetBody = strTargetBody;
testCase.TestData.bDEBUG_MODE = false;
testCase.TestData.charRootDirRadiometricRGB_RT = charRootDirRadiometricRGB_RT;
end

%%
function test_CRadiometricRGB_RT(testCase)
% Test with test_CRadiometricRGB_RT implementation
charRootDirRadiometricRGB_RT = testCase.TestData.charRootDirRadiometricRGB_RT;
charSceneConfigFilePath = fullfile(charRootDirRadiometricRGB_RT, 'tests', 'test_data', 'scene_configs', 'scene.yml');

if isfolder(charRootDirRadiometricRGB_RT)
    charRayTracerMatlabPath = fullfile(charRootDirRadiometricRGB_RT, "matlab");
    charRayTracerLibPath = fullfile(charRootDirRadiometricRGB_RT, "build", "wrap", "radiometric_cpu_raytracer");
    charRayTracerMexPath = fullfile(charRootDirRadiometricRGB_RT, "build", "wrap", "radiometric_cpu_raytracer_mex");

    if isfolder(charRayTracerMatlabPath)
        testCase.applyFixture(matlab.unittest.fixtures.PathFixture(charRayTracerMatlabPath, "IncludeSubfolders", true));
    end
    if isfolder(charRayTracerLibPath)
        testCase.applyFixture(matlab.unittest.fixtures.PathFixture(charRayTracerLibPath));
    end
    if isfolder(charRayTracerMexPath)
        testCase.applyFixture(matlab.unittest.fixtures.PathFixture(charRayTracerMexPath));
    end
end

charRTlibPath = which("radiometric_rgb_rt.CRadiometricRGB_RT");
bHasExternalRaytracer = not(isempty(charRTlibPath));

testCase.assumeTrue(bHasExternalRaytracer, ...
    "External radiometric RGB raytracer is unavailable, skipping test.");
testCase.assumeTrue(isfile(charSceneConfigFilePath), ...
    "Scene configuration file for radiometric raytracer is missing, skipping test.");

objShapeModel = testCase.TestData.objShapeModel;
dPointsPositionsGT_TB = testCase.TestData.dPointsPositionsGT_TB;
dCameraPosition_TB = testCase.TestData.dCameraPosition_TB;
dSunPosition_TB = testCase.TestData.dSunPosition_TB;
dDCM_TBfromCAM = testCase.TestData.dDCM_TBfromCAM;

% Initialize raytracer
objRayTracer = radiometric_rgb_rt.CRadiometricRGB_RT();
objRayTracer.configureFromYamlFile(char(charSceneConfigFilePath), false);

% Set camera data
objRayTracer.setCameraGeomParams(testCase.TestData.strCameraData.dResX, ...
                                 testCase.TestData.strCameraData.dResY, ...
                                 testCase.TestData.strCameraData.dKcam(1,1), ...
                                 testCase.TestData.strCameraData.dKcam(2,2), ...
                                 testCase.TestData.strCameraData.dKcam(1,3), ...
                                 testCase.TestData.strCameraData.dKcam(2,3), ...
                                 1, 0);

% Add mesh
objRayTracer.addTriaMeshToScene(objShapeModel.dVerticesPos, ...
                                double(objShapeModel.ui32triangVertexPtr));

dTestPointsPos_TB = dPointsPositionsGT_TB - dCameraPosition_TB;
dTestPointsDir_TB = dTestPointsPos_TB ./ vecnorm(dTestPointsPos_TB, 2, 1);

dIntersectPoints_TB = objRayTracer.rayTraceDirections(dSunPosition_TB, ...
                                                dCameraPosition_TB, ...
                                                dDCM_TBfromCAM, ...
                                                dTestPointsDir_TB);

% Test visibility using dedicated method
bVisibilityMask_visibilityMethod = logical(objRayTracer.rayTracePointsVisibility(dSunPosition_TB, ...
                                                    dCameraPosition_TB, ...
                                                    dDCM_TBfromCAM, ...
                                                    dPointsPositionsGT_TB));

% Compute depth of intersection points
dIntersectDistance = vecnorm(dIntersectPoints_TB, 2, 1);
bValidPoints = dIntersectDistance > eps('double');

% Reconstruct mask for visibility
dIntersectPointsFromCam     = dIntersectPoints_TB - dCameraPosition_TB;
dDistIntersectFromCam       = vecnorm(dIntersectPointsFromCam, 2, 1);
bIsPointVisibleAndInFront   = bValidPoints & (abs(dDistIntersectFromCam - vecnorm(dTestPointsPos_TB, 2, 1)) <= eps('single'));

dValidIntersectPoints = dIntersectPoints_TB(:, bValidPoints);
% dValidVisiblePoints_intersectMethod = dIntersectPoints_TB(:, bIsPointVisibleAndInFront);

dValidVisiblePoints_visibilityMethod = dPointsPositionsGT_TB(:, bVisibilityMask_visibilityMethod);

% Check numerical validity
testCase.verifyTrue(all(isfinite(dIntersectPoints_TB(:))), ...
    "Ray tracer returned non-finite intersection points.");
testCase.verifyTrue(any(bIsPointVisibleAndInFront), ...
    "Ray tracer did not find any visible points in front of the camera.");

% Verify sizes
testCase.verifySize(dIntersectPoints_TB, size(dPointsPositionsGT_TB));
testCase.verifySize(bVisibilityMask_visibilityMethod, [size(dPointsPositionsGT_TB, 2), 1]);
testCase.verifySize(dValidIntersectPoints, [3, nnz(bValidPoints)]);
testCase.verifySize(dValidVisiblePoints_visibilityMethod, [3, nnz(bVisibilityMask_visibilityMethod)]);

% Compare output between two cases
testCase.verifyTrue( abs(nnz(bIsPointVisibleAndInFront) - nnz(bVisibilityMask_visibilityMethod)) < floor(0.005 * length(bValidPoints)) );

% Rough timing 
fcnHandle_mex = @() objRayTracer.rayTraceDirections(dSunPosition_TB, ...
                                                dCameraPosition_TB, ...
                                                dDCM_TBfromCAM, ...
                                                dTestPointsDir_TB); 

[dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle_mex, 10);
end

%%
function test_RayTracePointVisibilityLocalPA_MEX(testCase)
% Test approximate function using raytracing with ellipsoid local assumption for illumination

% TODO update input structure for usage
bDEBUG_MODE = testCase.TestData.bDEBUG_MODE;
strTargetBodyData = orderfields(testCase.TestData.strTargetBodyData);
strCameraData = orderfields(testCase.TestData.strCameraData);
strFcnOptions = orderfields(testCase.TestData.strFcnOptions);
ui32VerticesIDs = testCase.TestData.ui32VerticesIDs;
dPointsPositionsGT_TB = testCase.TestData.dPointsPositionsGT_TB;
dSunPosition_TB = testCase.TestData.dSunPosition_TB;

testCase.assumeTrue(exist("RayTracePointVisibility_EllipsLocalPA_MEX", "file") > 0, ...
    "RayTracePointVisibility_EllipsLocalPA_MEX is unavailable, skipping test.");

fcnHandle_mex = @() RayTracePointVisibility_EllipsLocalPA_MEX(uint32(ui32VerticesIDs), dPointsPositionsGT_TB, ...
    strTargetBodyData, strCameraData, dSunPosition_TB, strFcnOptions, bDEBUG_MODE);

[dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle_mex, 10);

[bAllPointsVisibilityMask_legacyEllipsLocalPA, ~] = RayTracePointVisibility_EllipsLocalPA_MEX(uint32(ui32VerticesIDs), ...
                                                                                               dPointsPositionsGT_TB, ...
                                                                                               strTargetBodyData, ...
                                                                                               strCameraData, ...
                                                                                               dSunPosition_TB, ...
                                                                                               strFcnOptions, ...
                                                                                               bDEBUG_MODE);

testCase.verifyTrue(isnumeric(dAvgRunTime) && isscalar(dAvgRunTime));
testCase.verifyTrue(isnumeric(dTimings) && all(isfinite(dTimings(:))));
testCase.verifySize(bAllPointsVisibilityMask_legacyEllipsLocalPA, [size(dPointsPositionsGT_TB, 2), 1]);
testCase.verifyClass(bAllPointsVisibilityMask_legacyEllipsLocalPA, "logical");
end

%%
function test_RayTracePointVisibility_ShadowRays(testCase)
% Test approximate function using raytracing with shadow rays check

% TODO update input structure for usage
bTwoSidedTest = true;
bPointsAreMeshVertices = true;
bSkipIlluminationCheck = false;
bDEBUG_MODE = testCase.TestData.bDEBUG_MODE;

strTargetBodyData = orderfields(testCase.TestData.strTargetBodyData);
strCameraData = orderfields(testCase.TestData.strCameraData);
ui32VerticesIDs = testCase.TestData.ui32VerticesIDs;
dPointsPositionsGT_TB = testCase.TestData.dPointsPositionsGT_TB;
dSunPosition_TB = testCase.TestData.dSunPosition_TB;

testCase.assumeTrue(exist("RayTracePointVisibility_ShadowRays_MEX", "file") > 0, ...
    "RayTracePointVisibility_ShadowRays_MEX is unavailable, skipping test.");

fcnHandle_mex = @() RayTracePointVisibility_ShadowRays_MEX(uint32(ui32VerticesIDs), ...
                                                           dPointsPositionsGT_TB, ...
                                                           strTargetBodyData, ...
                                                           strCameraData, ...
                                                           dSunPosition_TB, ...
                                                           bDEBUG_MODE, ...
                                                           bTwoSidedTest, ...
                                                           bPointsAreMeshVertices, ...
                                                           bSkipIlluminationCheck); 


[dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle_mex, 10); %#ok<*ASGLU>
[bAllPointsVisibilityMask_RTwithShadowRays, ~] = fcnHandle_mex(); 

testCase.verifySize(bAllPointsVisibilityMask_RTwithShadowRays, [size(dPointsPositionsGT_TB, 2), 1]);
testCase.verifyClass(bAllPointsVisibilityMask_RTwithShadowRays, "logical");
testCase.verifyTrue(any(bAllPointsVisibilityMask_RTwithShadowRays), ...
    "Shadow ray tracing reported zero visible points.");
end

%%
function test_CRadiometricRGB_RT_vs_ShadowRays(testCase)
% Cross-validate external raytracer output with shadow rays MEX using same inputs
charRootDirRadiometricRGB_RT = testCase.TestData.charRootDirRadiometricRGB_RT;
charSceneConfigFilePath = fullfile(charRootDirRadiometricRGB_RT, 'tests', 'test_data', 'scene_configs', 'scene.yml');

if isfolder(charRootDirRadiometricRGB_RT)
    charRayTracerMatlabPath = fullfile(charRootDirRadiometricRGB_RT, "matlab");
    charRayTracerLibPath = fullfile(charRootDirRadiometricRGB_RT, "build", "wrap", "radiometric_cpu_raytracer");
    charRayTracerMexPath = fullfile(charRootDirRadiometricRGB_RT, "build", "wrap", "radiometric_cpu_raytracer_mex");

    if isfolder(charRayTracerMatlabPath)
        testCase.applyFixture(matlab.unittest.fixtures.PathFixture(charRayTracerMatlabPath, "IncludeSubfolders", true));
    end
    if isfolder(charRayTracerLibPath)
        testCase.applyFixture(matlab.unittest.fixtures.PathFixture(charRayTracerLibPath));
    end
    if isfolder(charRayTracerMexPath)
        testCase.applyFixture(matlab.unittest.fixtures.PathFixture(charRayTracerMexPath));
    end
end

charRTlibPath = which("radiometric_rgb_rt.CRadiometricRGB_RT");
bHasExternalRaytracer = not(isempty(charRTlibPath));

testCase.assumeTrue(bHasExternalRaytracer, ...
    "External radiometric RGB raytracer is unavailable, skipping test.");
testCase.assumeTrue(isfile(charSceneConfigFilePath), ...
    "Scene configuration file for radiometric raytracer is missing, skipping test.");
testCase.assumeTrue(exist("RayTracePointVisibility_ShadowRays_MEX", "file") > 0, ...
    "RayTracePointVisibility_ShadowRays_MEX is unavailable, skipping cross-validation test.");

objShapeModel = testCase.TestData.objShapeModel;
ui32VerticesIDs = testCase.TestData.ui32VerticesIDs;
dPointsPositionsGT_TB = testCase.TestData.dPointsPositionsGT_TB;
dCameraPosition_TB = testCase.TestData.dCameraPosition_TB;
dSunPosition_TB = testCase.TestData.dSunPosition_TB;
dDCM_TBfromCAM = testCase.TestData.dDCM_TBfromCAM;
strTargetBodyData = orderfields(testCase.TestData.strTargetBodyData);
strCameraData = orderfields(testCase.TestData.strCameraData);
bDEBUG_MODE = testCase.TestData.bDEBUG_MODE;

bTwoSidedTest = true;
bPointsAreMeshVertices = true;
bSkipIlluminationCheck = false;

%%% External raytracer visibility check
% Configure instance
objRayTracer = radiometric_rgb_rt.CRadiometricRGB_RT();
objRayTracer.configureFromYamlFile(char(charSceneConfigFilePath), false);
objRayTracer.setCameraGeomParams(strCameraData.dResX, ...
                                 strCameraData.dResY, ...
                                 strCameraData.dKcam(1,1), ...
                                 strCameraData.dKcam(2,2), ...
                                 strCameraData.dKcam(1,3), ...
                                 strCameraData.dKcam(2,3), ...
                                 1, 0);
objRayTracer.addTriaMeshToScene(objShapeModel.dVerticesPos, ...
                                double(objShapeModel.ui32triangVertexPtr));

bVisibilityMask_visibilityMethod = logical(objRayTracer.rayTracePointsVisibility(dSunPosition_TB, ...
                                                    dCameraPosition_TB, ...
                                                    dDCM_TBfromCAM, ...
                                                    dPointsPositionsGT_TB));
bVisibilityMask_visibilityMethod = bVisibilityMask_visibilityMethod(:);

% Shadow rays MEX visibility
[bAllPointsVisibilityMask_RTwithShadowRays, ~] = RayTracePointVisibility_ShadowRays_MEX(uint32(ui32VerticesIDs), ...
                                                                                         dPointsPositionsGT_TB, ...
                                                                                         strTargetBodyData, ...
                                                                                         strCameraData, ...
                                                                                         dSunPosition_TB, ...
                                                                                         bDEBUG_MODE, ...
                                                                                         bTwoSidedTest, ...
                                                                                         bPointsAreMeshVertices, ...
                                                                                         bSkipIlluminationCheck); 

bAllPointsVisibilityMask_RTwithShadowRays = bAllPointsVisibilityMask_RTwithShadowRays(:);

ui32NumPoints = size(dPointsPositionsGT_TB, 2);
testCase.verifySize(bVisibilityMask_visibilityMethod, [ui32NumPoints, 1]);
testCase.verifySize(bAllPointsVisibilityMask_RTwithShadowRays, [ui32NumPoints, 1]);

testCase.verifyTrue(any(bVisibilityMask_visibilityMethod), ...
    "External raytracer reported zero visible points.");
testCase.verifyTrue(any(bAllPointsVisibilityMask_RTwithShadowRays), ...
    "Shadow rays MEX reported zero visible points.");

ui32NumAgree = nnz(bVisibilityMask_visibilityMethod & bAllPointsVisibilityMask_RTwithShadowRays);
ui32NumUnion = nnz(bVisibilityMask_visibilityMethod | bAllPointsVisibilityMask_RTwithShadowRays);

testCase.verifyGreaterThan(ui32NumAgree, 0, ...
    "No overlap between external raytracer and shadow ray visibility masks.");

% Require reasonable agreement between the two methods
dAgreementRatio = double(ui32NumAgree) / double(ui32NumUnion);
testCase.verifyGreaterThan(dAgreementRatio, 0.90, ...
    "Visibility masks between external raytracer and shadow rays MEX disagree significantly.");
end
