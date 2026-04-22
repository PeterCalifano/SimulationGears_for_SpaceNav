classdef testPinholeProjectHP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for pinholeProjectHP (single-point pinhole projection).
    % Tests geometric correctness, principal point, behind-camera handling, and randomized consistency.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        % Typical camera intrinsic matrix (1024x1024 detector, focal length 50 px)
        dKcam = [50, 0, 512; 0, 50, 512; 0, 0, 1];
    end

    methods (Test)

        function testBoresightProjectsToPrincipalPoint(testCase)
            % A point on the camera boresight (z-axis in CAM frame) projects to the principal point
            dDCM_CAMfromIN = eye(3);         % Camera aligned with inertial
            dCamPos_IN     = [0; 0; 0];      % Camera at origin
            dPointPos_IN   = [0; 0; 10];     % Point along z-axis (boresight)

            [dPixCoord, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPointPos_IN);

            % Principal point is (cx, cy) = (512, 512)
            testCase.verifyEqual(dPixCoord, [512; 512], 'AbsTol', 1e-10, ...
                'Boresight point must project to the principal point');
        end

        function testKnownOffsetProjection(testCase)
            % Point offset from boresight by known amount should produce predictable pixel
            dDCM_CAMfromIN = eye(3);
            dCamPos_IN     = [0; 0; 0];
            % Point at (1, 0, 10) in CAM frame --> u = fx*X/Z + cx = 50*1/10 + 512 = 517
            dPointPos_IN   = [1; 0; 10];

            [dPixCoord, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPointPos_IN);

            dExpectedU = 50 * 1/10 + 512;  % 517
            dExpectedV = 50 * 0/10 + 512;  % 512
            testCase.verifyEqual(dPixCoord, [dExpectedU; dExpectedV], 'AbsTol', 1e-10);
        end

        function testBehindCameraFlag(testCase)
            % Point behind the camera (negative z in CAM) should produce w < 0
            dDCM_CAMfromIN = eye(3);
            dCamPos_IN     = [0; 0; 0];
            dPointPos_IN   = [0; 0; -10]; % Behind the camera

            [~, dPosPix] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPointPos_IN);

            testCase.verifyLessThan(dPosPix(3), 0, ...
                'Third component (w) must be negative for points behind the camera');
        end

        function testRotatedCamera(testCase)
            % Camera rotated 90 degrees about z-axis: what was x in IN becomes -y in CAM
            dDCM_CAMfromIN = [0, 1, 0; -1, 0, 0; 0, 0, 1]; % 90 deg rotation about z
            dCamPos_IN     = [0; 0; 0];
            dPointPos_IN   = [1; 0; 10]; % At (1,0,10) in IN --> (0,-1,10) in CAM? No: DCM*[1;0;10] = [0;-1;10]

            [dPixCoord, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPointPos_IN);

            % In CAM frame: (0, -1, 10), so u = 50*0/10+512 = 512, v = 50*(-1)/10+512 = 507
            dExpectedU = 512;
            dExpectedV = 50 * (-1)/10 + 512; % 507
            testCase.verifyEqual(dPixCoord, [dExpectedU; dExpectedV], 'AbsTol', 1e-10);
        end

        function testTranslatedCamera(testCase)
            % Camera offset from origin: point at origin should project differently
            dDCM_CAMfromIN = eye(3);
            dCamPos_IN     = [0; 0; -10]; % Camera 10 km behind origin
            dPointPos_IN   = [0; 0; 0];   % Point at origin

            % In CAM frame: point relative = DCM * (point - cam) = [0;0;10]
            [dPixCoord, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPointPos_IN);

            testCase.verifyEqual(dPixCoord, [512; 512], 'AbsTol', 1e-10, ...
                'Point at origin viewed from translated camera on boresight');
        end

        function testRandomPointsDistanceConsistency(testCase)
            % Points further from the camera should project closer to the principal point
            rng('default');
            dDCM_CAMfromIN = eye(3);
            dCamPos_IN     = [0; 0; 0];

            for idT = 1:20
                dLateralOffset = randn(2, 1) * 2;

                % Near point
                dPointNear = [dLateralOffset; 10];
                [dPixNear, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPointNear);

                % Far point (same lateral, 10x further)
                dPointFar = [dLateralOffset; 100];
                [dPixFar, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPointFar);

                % Far point should be closer to principal point
                dDistNear = norm(dPixNear - [512; 512]);
                dDistFar  = norm(dPixFar  - [512; 512]);

                testCase.verifyGreaterThan(dDistNear, dDistFar, ...
                    'Nearer points should project further from principal point');
            end
        end

        function testProjectionScalesLinearly(testCase)
            % Pixel displacement from principal point should scale as 1/Z for same lateral offset
            dDCM_CAMfromIN = eye(3);
            dCamPos_IN     = [0; 0; 0];

            dPoint1 = [2; 1; 20];
            dPoint2 = [2; 1; 40]; % Same lateral, twice the depth

            [dPix1, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPoint1);
            [dPix2, ~] = pinholeProjectHP(testCase.dKcam, dDCM_CAMfromIN, dCamPos_IN, dPoint2);

            dPrincipal = [512; 512];
            dDisp1 = dPix1 - dPrincipal;
            dDisp2 = dPix2 - dPrincipal;

            % disp2 should be exactly half of disp1
            testCase.verifyEqual(dDisp2, dDisp1 / 2, 'AbsTol', 1e-10, ...
                'Pixel displacement should scale inversely with depth');
        end

    end
end
