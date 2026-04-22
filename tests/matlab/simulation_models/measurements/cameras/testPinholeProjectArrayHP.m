classdef testPinholeProjectArrayHP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for pinholeProjectArrayHP (batch pinhole projection using JPL quaternion attitude).
    % Validates output dimensions, DCM orthogonality, consistency with pinholeProjectHP for each
    % point individually, boresight alignment, and depth-ordering of projections.
    % NOTE: pinholeProjectArrayHP uses [qv1; qv2; qv3; qs] quaternion storage (scalar last) internally.
    % Identity: [0; 0; 0; 1]. The user-facing convention preference is [qs; qv], but this function
    % departs from it -- quaternion arguments here must match the function's expected layout.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        % Typical normalized intrinsic matrix (focal length in pixels, principal point at image centre)
        dKcam = [800,   0, 320;
            0, 800, 240;
            0,   0,   1];
    end

    methods (Test)

        function testOutputDimensions(testCase)
            % Output pixel array must be [2xN] for N input points
            dNpoints = 7;
            dKcam    = testCase.dKcam;
            dqIdent  = [0; 0; 0; 1];          % identity rotation: [qv1;qv2;qv3;qs] JPL
            dRsc_IN  = [0; 0; 0];
            dPoints  = randn(3, dNpoints) + [0; 0; 10];  % in front of camera

            [dUV, dDCM] = pinholeProjectArrayHP(dKcam, dqIdent, dRsc_IN, dPoints);

            testCase.verifySize(dUV,  [2, dNpoints], 'Output pixel array must be [2xN]');
            testCase.verifySize(dDCM, [3, 3],         'Output DCM must be [3x3]');
        end

        function testReturnedDCMIsOrthogonal(testCase)
            % DCM returned must satisfy R*R' = I and det(R) = +1 for random attitudes
            dKcam = testCase.dKcam;
            rng('default');

            for idTrial = 1:15
                % Generate random unit quaternion (JPL: [qv; qs])
                dQ = randn(4, 1);
                dQ = dQ / norm(dQ);

                dPoints = randn(3, 1) + [0; 0; 5];
                [~, dDCM] = pinholeProjectArrayHP(dKcam, dQ, zeros(3,1), dPoints);

                % Orthogonality: R*R' = I
                dErr = norm(dDCM * dDCM' - eye(3), 'fro');
                testCase.verifyLessThan(dErr, 1e-12, ...
                    sprintf('DCM not orthogonal at trial %d (err=%.2e)', idTrial, dErr));

                % Proper rotation: det = +1
                testCase.verifyEqual(det(dDCM), 1.0, 'AbsTol', 1e-12, ...
                    sprintf('DCM determinant must be +1 at trial %d', idTrial));
            end
        end

        function testConsistencyWithSinglePointProjection(testCase)
            % For each point in the batch, pinholeProjectArrayHP must match pinholeProjectHP
            % when called with the DCM that pinholeProjectArrayHP returns.
            dKcam   = testCase.dKcam;
            dRsc_IN = [100; -50; 200];
            rng(42);

            for idTrial = 1:10
                % Random unit quaternion (JPL)
                dQ = randn(4, 1);
                dQ = dQ / norm(dQ);

                dNpoints = 5 + randi(10);
                % Points must be in front of the camera (positive depth after rotation)
                dPoints  = randn(3, dNpoints) * 50 + [0; 0; 800];

                [dUV_array, dDCM] = pinholeProjectArrayHP(dKcam, dQ, dRsc_IN, dPoints);

                for idP = 1:dNpoints
                    dUV_single = pinholeProjectHP(dKcam, dDCM, dRsc_IN, dPoints(:, idP));
                    testCase.verifyEqual(dUV_array(:, idP), dUV_single, 'AbsTol', 1e-10, ...
                        sprintf('Array vs single-point mismatch at trial %d, point %d', idTrial, idP));
                end
            end
        end

        function testBoresightProjectsToImageCentre(testCase)
            % A point on the camera boresight (z-axis in CAM frame) must project to
            % the principal point [cx; cy] = [Kcam(1,3); Kcam(2,3)]
            dKcam    = testCase.dKcam;
            dqIdent  = [0; 0; 0; 1];  % identity rotation: CAM aligned with IN
            dRsc_IN  = [0; 0; 0];

            % Points along boresight (+z in CAM = +z in IN for identity rotation)
            dDepths  = [5, 10, 50, 200];
            dPoints  = [zeros(2, numel(dDepths)); dDepths];

            [dUV, ~] = pinholeProjectArrayHP(dKcam, dqIdent, dRsc_IN, dPoints);

            dExpectedUV = repmat([dKcam(1,3); dKcam(2,3)], 1, numel(dDepths));
            testCase.verifyEqual(dUV, dExpectedUV, 'AbsTol', 1e-10, ...
                'Boresight points must project to principal point');
        end

        function testDepthOrderPreserved(testCase)
            % Points further from the camera still project to the same pixel if collinear
            % with the boresight; confirms no depth-dependent distortion
            dKcam    = testCase.dKcam;
            dqIdent  = [0; 0; 0; 1];
            dRsc_IN  = [0; 0; 0];

            % Two points at different depths but same angular direction
            dDirection = [1; 0.5; 10];
            dDir       = dDirection / norm(dDirection);
            dP1 = dDir * 10;
            dP2 = dDir * 100;

            [dUV, ~] = pinholeProjectArrayHP(dKcam, dqIdent, dRsc_IN, [dP1, dP2]);

            % Pinhole projection is direction-based: same direction --> same pixel
            testCase.verifyEqual(dUV(:, 1), dUV(:, 2), 'AbsTol', 1e-8, ...
                'Collinear points at different depths must project to the same pixel');
        end

        function testRandomAttitudesPixelsFiniteAndConsistent(testCase)
            % For 50 random scenes, all projected pixel coordinates must be finite and
            % the array result must exactly match the point-by-point result
            dKcam = testCase.dKcam;
            rng(99);

            for idTrial = 1:50
                dQ      = randn(4, 1); dQ = dQ / norm(dQ);
                dRsc_IN = randn(3, 1) * 100;
                dNpts   = randi(8) + 1;

                % Place points well in front of any possible camera orientation
                % by putting them far along their nominal boresight
                dPoints = randn(3, dNpts) * 20 + [0; 0; 500];

                [dUV, dDCM] = pinholeProjectArrayHP(dKcam, dQ, dRsc_IN, dPoints);

                testCase.verifyTrue(all(isfinite(dUV(:))), ...
                    sprintf('Non-finite pixel at trial %d', idTrial));

                for idP = 1:dNpts
                    dUV_ref = pinholeProjectHP(dKcam, dDCM, dRsc_IN, dPoints(:, idP));
                    testCase.verifyEqual(dUV(:, idP), dUV_ref, 'AbsTol', 1e-10, ...
                        sprintf('Consistency failure at trial %d point %d', idTrial, idP));
                end
            end
        end

        function testSinglePointInputGives2x1Output(testCase)
            % Edge case: a single point (3x1 input) must produce a [2x1] output
            dKcam   = testCase.dKcam;
            dqIdent = [0; 0; 0; 1];
            dRsc_IN = zeros(3, 1);
            dPoint  = [10; -5; 100];

            [dUV, ~] = pinholeProjectArrayHP(dKcam, dqIdent, dRsc_IN, dPoint);
            testCase.verifySize(dUV, [2, 1]);
        end

    end
end
