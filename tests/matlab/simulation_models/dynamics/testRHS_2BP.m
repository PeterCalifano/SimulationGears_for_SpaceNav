classdef testRHS_2BP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for RHS_2BP (Two-Body Problem right-hand side).
    % Tests include analytical verification, energy conservation, and randomized checks.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth = 398600.4418;    % Earth gravitational parameter [km^3/s^2]
        dRadiusLEO = 6871.0;       % Typical LEO radius [km]
    end

    methods (Test)

        function testCircularOrbitPeriod(testCase)
            % Propagate a circular orbit and verify the period matches T = 2*pi*sqrt(a^3/mu)
            dSma = testCase.dRadiusLEO;
            dExpectedPeriod = 2 * pi * sqrt(dSma^3 / testCase.dMuEarth);

            strDynParams.dGravParam = testCase.dMuEarth;
            dVcirc = sqrt(testCase.dMuEarth / dSma);
            dState0 = [dSma; 0; 0; 0; dVcirc; 0];

            % Propagate for one full period
            objOdeOpts = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);
            [~, dTrajectory] = ode113(@(dTime, dState) RHS_2BP(dTime, dState, strDynParams), ...
                [0, dExpectedPeriod], dState0, objOdeOpts);

            % After one period, position should return to initial
            dFinalPos = dTrajectory(end, 1:3)';
            dPosError = norm(dFinalPos - dState0(1:3));
            testCase.verifyLessThan(dPosError, 1e-6, ...
                'Position after one orbital period should return to initial (within 1e-6 km)');
        end

        function testEnergyConservation(testCase)
            % Verify specific mechanical energy is conserved along trajectory
            strDynParams.dGravParam = testCase.dMuEarth;
            dSma = testCase.dRadiusLEO;
            dVcirc = sqrt(testCase.dMuEarth / dSma);
            dState0 = [dSma; 0; 0; 0; dVcirc; 0];

            objOdeOpts = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);
            dPeriod = 2 * pi * sqrt(dSma^3 / testCase.dMuEarth);
            [~, dTrajectory] = ode113(@(dTime, dState) RHS_2BP(dTime, dState, strDynParams), ...
                linspace(0, dPeriod, 100), dState0, objOdeOpts);

            % Compute energy at each point: E = v^2/2 - mu/r
            dEnergy = zeros(size(dTrajectory, 1), 1);
            for idT = 1:size(dTrajectory, 1)
                dPosNorm = norm(dTrajectory(idT, 1:3));
                dVelNorm = norm(dTrajectory(idT, 4:6));
                dEnergy(idT) = 0.5 * dVelNorm^2 - testCase.dMuEarth / dPosNorm;
            end

            dRelEnergyDrift = abs(dEnergy - dEnergy(1)) / abs(dEnergy(1));
            testCase.verifyLessThan(max(dRelEnergyDrift), 1e-10, ...
                'Relative energy drift must be below 1e-10');
        end

        function testAccelerationMagnitude(testCase)
            % Verify |a| = mu/r^2 for the gravitational acceleration
            strDynParams.dGravParam = testCase.dMuEarth;
            dSma = testCase.dRadiusLEO;
            dVcirc = sqrt(testCase.dMuEarth / dSma);
            dState = [dSma; 0; 0; 0; dVcirc; 0];

            dxdt = RHS_2BP(0, dState, strDynParams);
            dAccMag = norm(dxdt(4:6));
            dExpectedAccMag = testCase.dMuEarth / dSma^2;

            testCase.verifyEqual(dAccMag, dExpectedAccMag, 'RelTol', 1e-12);
        end

        function testRandomOrbitsEnergyConservation(testCase)
            % Randomized test: energy conservation for random initial conditions
            rng('default');
            ui32NumTrials = 10;
            strDynParams.dGravParam = testCase.dMuEarth;

            for idT = 1:ui32NumTrials
                % Random orbit with radius 7000-40000 km
                dRadius = 7000 + 33000 * rand();
                dVcirc = sqrt(testCase.dMuEarth / dRadius);
                % Random direction for position and perpendicular velocity
                dRandDir = randn(3, 1);
                dRandDir = dRandDir / norm(dRandDir);
                dPosVec = dRadius * dRandDir;

                % Velocity perpendicular to position (with random magnitude 0.8-1.2 Vcirc)
                dVelDir = cross(dRandDir, randn(3, 1));
                dVelDir = dVelDir / norm(dVelDir);
                dVelMag = dVcirc * (0.8 + 0.4 * rand());
                dVelVec = dVelMag * dVelDir;

                dState0 = [dPosVec; dVelVec];
                dEnergy0 = 0.5 * dVelMag^2 - testCase.dMuEarth / dRadius;

                % Short propagation
                objOpts = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);
                [~, dTraj] = ode113(@(dTime, dState) RHS_2BP(dTime, dState, strDynParams), ...
                    [0, 3600], dState0, objOpts);

                dEnergyEnd = 0.5 * norm(dTraj(end, 4:6))^2 - testCase.dMuEarth / norm(dTraj(end, 1:3));
                dRelDrift = abs(dEnergyEnd - dEnergy0) / abs(dEnergy0);

                testCase.verifyLessThan(dRelDrift, 1e-10, ...
                    sprintf('Energy drift too large for trial %d', idT));
            end
        end

        function testAccelerationDirectionOpposesPosition(testCase)
            % Gravitational acceleration must always point toward the origin
            rng(42);
            strDynParams.dGravParam = testCase.dMuEarth;

            for idT = 1:20
                dPosVec = randn(3, 1) * 10000;
                dVelVec = randn(3, 1) * 5;
                dState = [dPosVec; dVelVec];

                dxdt = RHS_2BP(0, dState, strDynParams);
                dAccVec = dxdt(4:6);

                % Acceleration should be anti-parallel to position
                dDotProduct = dot(dAccVec, dPosVec);
                testCase.verifyLessThan(dDotProduct, 0, ...
                    'Gravitational acceleration must point toward the origin');
            end
        end

    end
end
