classdef testRayTracePointVisibilityShadowRays < matlab.unittest.TestCase
%% DESCRIPTION
% Visibility rule of RayTracePointVisibility_ShadowRays with mesh vertices as points (two-sided camera test, the
% setting of the nav-frontend emulators). A point in the field of view is visible when
%   1) the camera ray C -> P crosses no triangle in front of P (closer than P by more than eps('single')), and
%   2) the illumination test is skipped, or the shadow ray P -> Sun crosses no triangle (beyond eps('single')).
% Triangles that contain P are excluded from both tests. The cases:
%   - a night-side limb vertex whose camera ray crosses no triangle at all: invisible with a Sun behind the body,
%     visible with a Sun on its side and visible without the illumination test (the no-intersection branch used
%     to mark the point visible without the shadow test);
%   - a vertex whose camera ray crosses a triangle beyond P: visible when lit, invisible when the shadow ray is
%     blocked (behaviour kept);
%   - a vertex hidden behind the body: invisible in every mode;
%   - a crossing beyond P listed before an occluder in front of P: invisible also without the illumination test
%     (the loop used to stop at the first crossing beyond P when the illumination test was skipped);
%   - a star-shaped non-convex mesh: the mask equals an independent double-precision evaluation of the rule,
%     with and without a Sun.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-10-2026  Pietro Califano, Claude Opus 5.5  First implementation: illumination test of points whose camera
%                                               ray crosses no triangle.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SetupSimGears, RayTracePointVisibility_ShadowRays.
% -------------------------------------------------------------------------------------------------------------

properties
    charOriginalPath
end

properties (Constant)
    % Camera intrinsics shared by the scenes: 640 x 480 pixels
    dKcam = [800, 0, 320; 0, 800, 240; 0, 0, 1]
    dResX = 640
    dResY = 480
    % Sun distance of the scenes [same unit as the mesh]
    dSunDistance = 1e9
end

methods (TestClassSetup)
    function addSimulationGearsPaths(testCase)
        testCase.charOriginalPath = path;
        charRepoRoot = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', '..', '..');
        addpath(fullfile(charRepoRoot, 'matlab'));
        SetupSimGears;
        testCase.addTeardown(@path, testCase.charOriginalPath);
    end
end

methods (Test)
    function testNightSideLimbVertexIsNotIlluminated(testCase)
        % +x is on the limb seen from the camera: its camera ray touches the body only at P. The Sun behind the
        % body blocks its shadow ray, which crosses the face (-x, +y, +z) at s = 2 / 1.3 (interior point).
        [strTargetBodyData, strCameraData, dVertices_TB] = testCase.buildOctahedronScene_();
        ui32Limb = uint32(1);
        testCase.assertFalse(CameraRayCrossesAnyTriangle_(strCameraData.dPosition_IN, dVertices_TB, ...
            strTargetBodyData.strShapeModel.ui32triangVertexPtr, ui32Limb), 'Scene: the limb ray must cross nothing.');
        dSunNight_TB = testCase.dSunDistance * [-1; 0.1; 0.2] / norm([-1; 0.1; 0.2]);

        bWithSun = RayTracePointVisibility_ShadowRays(ui32Limb, dVertices_TB(:, ui32Limb), strTargetBodyData, ...
            strCameraData, dSunNight_TB, false, true, true, false);
        bWithoutSun = RayTracePointVisibility_ShadowRays(ui32Limb, dVertices_TB(:, ui32Limb), strTargetBodyData, ...
            strCameraData, dSunNight_TB, false, true, true, true);

        testCase.verifyFalse(bWithSun, 'Unlit limb vertex must be invisible when the illumination test runs.');
        testCase.verifyTrue(bWithoutSun, 'Limb vertex must be visible without the illumination test.');
    end

    function testLitLimbVertexStaysVisible(testCase)
        % Same limb vertex with the Sun on its side: the shadow ray leaves the body (x > 1 along the ray).
        [strTargetBodyData, strCameraData, dVertices_TB] = testCase.buildOctahedronScene_();
        ui32Limb = uint32(1);
        dSunLit_TB = testCase.dSunDistance * [1; 0.3; 0.5] / norm([1; 0.3; 0.5]);

        [bVisible, dProjected_UV] = RayTracePointVisibility_ShadowRays(ui32Limb, dVertices_TB(:, ui32Limb), ...
            strTargetBodyData, strCameraData, dSunLit_TB, false, true, true, false);

        testCase.verifyTrue(bVisible);
        testCase.verifyEqual(dProjected_UV, ...
            ProjectPoints_(testCase.dKcam, strCameraData, dVertices_TB(:, ui32Limb)), ...
            'AbsTol', 1e-9);
    end

    function testPointWithCrossingBeyondKeepsShadowTest(testCase)
        % +z faces the camera; its camera ray crosses the face (-x, -y, -z) beyond P. Lit from above it is visible;
        % lit from below its shadow ray crosses the face (+x, +y, -z) and it is invisible.
        [strTargetBodyData, strCameraData, dVertices_TB] = testCase.buildOctahedronScene_();
        ui32Top = uint32(5);
        testCase.assertTrue(CameraRayCrossesAnyTriangle_(strCameraData.dPosition_IN, dVertices_TB, ...
            strTargetBodyData.strShapeModel.ui32triangVertexPtr, ui32Top), 'Scene: the top ray must cross the body.');
        dSunAbove_TB = testCase.dSunDistance * [0.1; 0.2; 1] / norm([0.1; 0.2; 1]);
        dSunBelow_TB = testCase.dSunDistance * [0.1; 0.2; -1] / norm([0.1; 0.2; -1]);

        bLit = RayTracePointVisibility_ShadowRays(ui32Top, dVertices_TB(:, ui32Top), strTargetBodyData, ...
            strCameraData, dSunAbove_TB, false, true, true, false);
        bUnlit = RayTracePointVisibility_ShadowRays(ui32Top, dVertices_TB(:, ui32Top), strTargetBodyData, ...
            strCameraData, dSunBelow_TB, false, true, true, false);

        testCase.verifyTrue(bLit);
        testCase.verifyFalse(bUnlit);
    end

    function testHiddenVertexIsInvisibleInEveryMode(testCase)
        % -z is behind the body seen from the camera: the camera ray crosses a top face in front of P.
        [strTargetBodyData, strCameraData, dVertices_TB] = testCase.buildOctahedronScene_();
        ui32Bottom = uint32(6);
        dSunAbove_TB = testCase.dSunDistance * [0.1; 0.2; 1] / norm([0.1; 0.2; 1]);
        dSunBelow_TB = testCase.dSunDistance * [0.1; 0.2; -1] / norm([0.1; 0.2; -1]);

        for bSkipIllumination = [false, true]
            for dSun_TB = [dSunAbove_TB, dSunBelow_TB]
                bVisible = RayTracePointVisibility_ShadowRays(ui32Bottom, dVertices_TB(:, ui32Bottom), ...
                    strTargetBodyData, strCameraData, dSun_TB, false, true, true, bSkipIllumination);
                testCase.verifyFalse(bVisible);
            end
        end
    end

    function testOccluderAfterCrossingBeyondPoint(testCase)
        % P = (0, 0, 0) is a vertex of triangle 1. Triangle 2 lies beyond P (plane z = -1), triangle 3 in front of
        % P (plane z = 5). Both cross the camera ray; triangle 3 hides P whatever the order of the triangles.
        dVertices_TB = [0, 1, 0, -3, 3, 0, -3, 3, 0; ...
                        0, 0, 1, -3, -3, 3, -3, -3, 3; ...
                        0, 0, 0, -1, -1, -1, 5, 5, 5];
        ui32Triangles = uint32([1, 4, 7; 2, 5, 8; 3, 6, 9]);
        strTargetBodyData = BuildTargetBodyData_(dVertices_TB, ui32Triangles);
        strCameraData = BuildLookAtCamera_([0.05; 0.03; 10], [0; 0; 0], testCase.dKcam, testCase.dResX, testCase.dResY);
        dSunAbove_TB = testCase.dSunDistance * [0; 0.1; 1] / norm([0; 0.1; 1]);

        for bSkipIllumination = [false, true]
            bVisible = RayTracePointVisibility_ShadowRays(uint32(1), dVertices_TB(:, 1), strTargetBodyData, ...
                strCameraData, dSunAbove_TB, false, true, true, bSkipIllumination);
            testCase.verifyFalse(bVisible, sprintf('bSkipIlluminationCheck = %d', bSkipIllumination));
        end
    end

    function testNonConvexMeshMatchesIndependentRule(testCase)
        % Star-shaped mesh with radial bumps: limb points, self-shadowed points and points hidden by bumps. The
        % mask must equal the rule of the description evaluated in double precision over all triangles.
        [dVertices_TB, ui32Triangles] = BuildBumpyMesh_(uint32(240));
        strTargetBodyData = BuildTargetBodyData_(dVertices_TB, ui32Triangles);
        strCameraData = BuildLookAtCamera_([1.1; 0.7; 6.5], [0; 0; 0], testCase.dKcam, testCase.dResX, testCase.dResY);
        dSun_TB = testCase.dSunDistance * [-0.6; 0.2; 1] / norm([-0.6; 0.2; 1]);
        ui32PointsIdx = uint32(1:size(dVertices_TB, 2));

        for bSkipIllumination = [false, true]
            bVisible = RayTracePointVisibility_ShadowRays(ui32PointsIdx, dVertices_TB, strTargetBodyData, ...
                strCameraData, dSun_TB, false, true, true, bSkipIllumination);
            [bExpected, strCounts] = EvaluateVisibilityRule_(dVertices_TB, ui32Triangles, strCameraData, ...
                testCase.dKcam, dSun_TB, bSkipIllumination);
            testCase.verifyEqual(bVisible(:)', bExpected, sprintf('bSkipIlluminationCheck = %d', bSkipIllumination));
            % The scene must exercise the decisions: hidden points, unlit points and unlit points whose camera ray
            % crosses no triangle at all.
            testCase.verifyGreaterThan(strCounts.ui32NumHidden, 0);
            if ~bSkipIllumination
                testCase.verifyGreaterThan(strCounts.ui32NumUnlit, 0);
                testCase.verifyGreaterThan(strCounts.ui32NumUnlitWithoutCameraCrossing, 0);
            end
        end
    end
end

methods (Access = private)
    function [strTargetBodyData, strCameraData, dVertices_TB] = buildOctahedronScene_(testCase)
        % Unit octahedron (vertices +x, -x, +y, -y, +z, -z) seen from (0.2, 0.1, 10), camera aimed at the origin.
        dVertices_TB = [1, -1, 0, 0, 0, 0; 0, 0, 1, -1, 0, 0; 0, 0, 0, 0, 1, -1];
        ui32Triangles = zeros(3, 0, 'uint32');
        for ui32X = uint32([1, 2])
            for ui32Y = uint32([3, 4])
                for ui32Z = uint32([5, 6])
                    ui32Triangles(:, end + 1) = [ui32X; ui32Y; ui32Z]; %#ok<AGROW>
                end
            end
        end
        strTargetBodyData = BuildTargetBodyData_(dVertices_TB, ui32Triangles);
        strCameraData = BuildLookAtCamera_([0.2; 0.1; 10], [0; 0; 0], testCase.dKcam, testCase.dResX, testCase.dResY);
    end
end
end

%% Local functions
function strTargetBodyData = BuildTargetBodyData_(dVertices_TB, ui32Triangles)
% Target body aligned with the inertial frame.
strTargetBodyData = struct('strShapeModel', struct('dVerticesPos', dVertices_TB, ...
    'ui32triangVertexPtr', uint32(ui32Triangles)), 'dDCM_INfromTB', eye(3));
end

function strCameraData = BuildLookAtCamera_(dPosition_IN, dTarget_IN, dKcam, dResX, dResY)
% Camera boresight (+z) towards the target, +x orthogonal to the inertial +y axis.
dBoresight = (dTarget_IN - dPosition_IN) / norm(dTarget_IN - dPosition_IN);
dAxisX = cross([0; 1; 0], dBoresight);
dAxisX = dAxisX / norm(dAxisX);
dDCM_INfromCAM = [dAxisX, cross(dBoresight, dAxisX), dBoresight];
strCameraData = struct('dDCM_INfromCAM', dDCM_INfromCAM, ...
                       'dPosition_IN', dPosition_IN, ...
                       'dResX', dResX, ...
                       'dResY', dResY, ...
                       'dKcam', dKcam);
end

function dProjected_UV = ProjectPoints_(dKcam, strCameraData, dPoints_TB)
% Pinhole projection of points given in the target frame (aligned with the inertial frame).
dPoints_CAM = strCameraData.dDCM_INfromCAM' * (dPoints_TB - strCameraData.dPosition_IN);
dHomogeneous = dKcam * dPoints_CAM;
dProjected_UV = dHomogeneous(1:2, :) ./ dHomogeneous(3, :);
end

function bCrosses = CameraRayCrossesAnyTriangle_(dCamera_TB, dVertices_TB, ui32Triangles, ui32Point)
% True when the ray from the camera through vertex ui32Point crosses a triangle not containing it (any t > 0).
dPoint = dVertices_TB(:, ui32Point);
bOther = ~any(ui32Triangles == ui32Point, 1);
[bHit, ~] = IntersectAllTriangles_(dCamera_TB, (dPoint - dCamera_TB) / norm(dPoint - dCamera_TB), ...
    dVertices_TB, ui32Triangles(:, bOther), eps);
bCrosses = any(bHit);
end

function [bHit, dT] = IntersectAllTriangles_(dOrigin, dDirection, dVertices_TB, ui32Triangles, dMinRange)
% Double-precision two-sided Moller-Trumbore against every triangle: hit when the barycentric coordinates are in
% [0, 1], |det| >= eps and t > dMinRange.
dA = dVertices_TB(:, ui32Triangles(1, :));
dEdge1 = dVertices_TB(:, ui32Triangles(2, :)) - dA;
dEdge2 = dVertices_TB(:, ui32Triangles(3, :)) - dA;
dP = cross(repmat(dDirection, 1, size(dA, 2)), dEdge2, 1);
dDeterminant = sum(dEdge1 .* dP, 1);
dS = dOrigin - dA;
dU = sum(dS .* dP, 1) ./ dDeterminant;
dQ = cross(dS, dEdge1, 1);
dV = sum(dDirection .* dQ, 1) ./ dDeterminant;
dT = sum(dEdge2 .* dQ, 1) ./ dDeterminant;
bHit = abs(dDeterminant) >= eps & dU >= 0 & dU <= 1 & dV >= 0 & dU + dV <= 1 & dT > dMinRange;
end

function [bExpected, strCounts] = EvaluateVisibilityRule_(dVertices_TB, ui32Triangles, strCameraData, dKcam, ...
    dSun_TB, bSkipIllumination)
% Rule of the description: in the field of view, no camera-ray crossing in front of P by more than eps('single'),
% and (illumination skipped or no shadow-ray crossing beyond eps('single')); triangles at P excluded.
dCamera_TB = strCameraData.dPosition_IN;
dProjected_UV = ProjectPoints_(dKcam, strCameraData, dVertices_TB);
bInView = dProjected_UV(1, :) > 0 & dProjected_UV(1, :) < strCameraData.dResX & ...
    dProjected_UV(2, :) > 0 & dProjected_UV(2, :) < strCameraData.dResY;
dNumPoints = size(dVertices_TB, 2);
bExpected = false(1, dNumPoints);
strCounts = struct('ui32NumHidden', uint32(0), ...
                   'ui32NumUnlit', uint32(0), ...
                   'ui32NumUnlitWithoutCameraCrossing', uint32(0));
for dPointIdx = 1:dNumPoints
    if ~bInView(dPointIdx)
        continue
    end
    dPoint = dVertices_TB(:, dPointIdx);
    ui32OtherTriangles = ui32Triangles(:, ~any(ui32Triangles == dPointIdx, 1));
    dRange = norm(dPoint - dCamera_TB);
    [bCameraHit, dCameraT] = IntersectAllTriangles_(dCamera_TB, (dPoint - dCamera_TB) / dRange, dVertices_TB, ...
        ui32OtherTriangles, eps);
    if any(bCameraHit & dRange - dCameraT > eps('single'))
        strCounts.ui32NumHidden = strCounts.ui32NumHidden + 1;
        continue
    end
    if bSkipIllumination
        bExpected(dPointIdx) = true;
        continue
    end
    dToSun = dSun_TB - dPoint;
    bShadowHit = IntersectAllTriangles_(dPoint, dToSun / norm(dToSun), dVertices_TB, ui32OtherTriangles, ...
        double(eps('single')));
    bExpected(dPointIdx) = ~any(bShadowHit);
    if any(bShadowHit)
        strCounts.ui32NumUnlit = strCounts.ui32NumUnlit + 1;
        strCounts.ui32NumUnlitWithoutCameraCrossing = strCounts.ui32NumUnlitWithoutCameraCrossing + ...
            uint32(~any(bCameraHit));
    end
end
end

function [dVertices_TB, ui32Triangles] = BuildBumpyMesh_(ui32NumPoints)
% Fibonacci points on the unit sphere triangulated by their convex hull, then moved radially by
% r = 1 + 0.25 sin(3 lon) sin(2 lat) + 0.15 cos(5 lat): a star-shaped, non-convex closed mesh.
dIndex = (0:double(ui32NumPoints) - 1) + 0.5;
dLatitude = asin(1 - 2 * dIndex / double(ui32NumPoints));
dLongitude = pi * (1 + sqrt(5)) * dIndex;
dUnit = [cos(dLatitude) .* cos(dLongitude); cos(dLatitude) .* sin(dLongitude); sin(dLatitude)];
ui32Triangles = uint32(convhull(dUnit')');
dRadius = 1 + 0.25 * sin(3 * dLongitude) .* sin(2 * dLatitude) + 0.15 * cos(5 * dLatitude);
dVertices_TB = dUnit .* dRadius;
end
