classdef testComputeJ2Perturbation_Gauss < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for ComputeJ2Perturbation_Gauss (J2 perturbation acceleration in RSW frame).
    % Includes dimensional checks, physical constraints, scaling laws, and randomized tests.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth    = 398600.4418;   % [km^3/s^2]
        dRadiusEarth = 6378.137;     % [km]
        dJ2Earth    = 1.08263e-3;    % [-]
    end

    methods (Test)

        function testOutputDimension(testCase)
            % Output must be a 3x1 column vector
            dResult = ComputeJ2Perturbation_Gauss(7000, 0.01, deg2rad(45), deg2rad(30), ...
                                           0, testCase.dRadiusEarth, testCase.dMuEarth, testCase.dJ2Earth);
            testCase.verifySize(dResult, [3, 1]);
        end

        function testZeroJ2GivesZeroAcceleration(testCase)
            % With J2 = 0, the perturbation must vanish
            dResult = ComputeJ2Perturbation_Gauss(7000, 0.01, deg2rad(45), deg2rad(30), ...
                                           deg2rad(60), testCase.dRadiusEarth, testCase.dMuEarth, 0);
            testCase.verifyEqual(dResult, zeros(3, 1), 'AbsTol', 1e-20, ...
                'J2 = 0 must produce zero perturbation');
        end

        function testMagnitudeScalingWithRadius(testCase)
            % J2 perturbation magnitude scales as 1/r^4 (from common factor)
            dTrueAnom = deg2rad(30);
            dIncl     = deg2rad(45);
            dArgPeri  = deg2rad(60);
            dEcc      = 0.01;

            dSma1 = 7000;
            dSma2 = 14000;

            dAcc1 = ComputeJ2Perturbation_Gauss(dSma1, dEcc, dIncl, dArgPeri, dTrueAnom, ...
                                          testCase.dRadiusEarth, testCase.dMuEarth, testCase.dJ2Earth);
            dAcc2 = ComputeJ2Perturbation_Gauss(dSma2, dEcc, dIncl, dArgPeri, dTrueAnom, ...
                                          testCase.dRadiusEarth, testCase.dMuEarth, testCase.dJ2Earth);

            % For near-circular orbit, r ~ a, so |acc| ratio ~ (a2/a1)^4
            dExpectedRatio = (dSma1/dSma2)^4;
            dActualRatio = norm(dAcc1) / norm(dAcc2);

            testCase.verifyEqual(dActualRatio, dExpectedRatio, 'RelTol', 0.05, ...
                'J2 magnitude should scale approximately as 1/r^4');
        end

        function testEquatorialOrbitNoCrossTrack(testCase)
            % At zero inclination, the cross-track (W) component must vanish
            dResult = ComputeJ2Perturbation_Gauss(7000, 0.0, 0, deg2rad(30), deg2rad(45), ...
                                           testCase.dRadiusEarth, testCase.dMuEarth, testCase.dJ2Earth);
            testCase.verifyEqual(dResult(3), 0, 'AbsTol', 1e-20, ...
                'Cross-track J2 perturbation must be zero for equatorial orbit');
        end

        function testPolarOrbitMaxCrossTrack(testCase)
            % At i=90 deg, cross-track W is proportional to sin(2*i)*sin(u) = 0 since sin(180)=0
            % Actually sin(2*90) = sin(180) = 0, so W vanishes for polar orbit too
            dResult = ComputeJ2Perturbation_Gauss(7000, 0.0, pi/2, deg2rad(30), deg2rad(45), ...
                                           testCase.dRadiusEarth, testCase.dMuEarth, testCase.dJ2Earth);
            testCase.verifyEqual(dResult(3), 0, 'AbsTol', 1e-20, ...
                'Cross-track J2 perturbation must vanish at polar inclination (sin(2*pi/2)=0)');
        end

        function testRandomStatesPhysicalBounds(testCase)
            % Randomized: J2 perturbation magnitude must be much smaller than central gravity
            rng('default');
            for idT = 1:20
                dSma      = 6800 + 30000 * rand();
                dEcc      = 0.3 * rand();
                dIncl     = pi * rand();
                dArgPeri  = 2*pi * rand();
                dTrueAnom = 2*pi * rand();

                dAccJ2 = ComputeJ2Perturbation_Gauss(dSma, dEcc, dIncl, dArgPeri, dTrueAnom, ...
                                               testCase.dRadiusEarth, testCase.dMuEarth, testCase.dJ2Earth);

                % Central gravity magnitude at this orbit
                dSemiLatRect = dSma * (1 - dEcc^2);
                dRadius = dSemiLatRect / (1 + dEcc * cos(dTrueAnom));
                dCentralAcc = testCase.dMuEarth / dRadius^2;

                % J2 perturbation should be orders of magnitude smaller than central gravity
                dRatio = norm(dAccJ2) / dCentralAcc;
                testCase.verifyLessThan(dRatio, 0.01, ...
                    sprintf('J2/central ratio too large at trial %d: %.4e', idT, dRatio));
            end
        end

    end
end
