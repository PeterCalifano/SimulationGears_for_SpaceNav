classdef testRHS_CR3BP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for RHS_CR3BP (Circular Restricted Three-Body Problem right-hand side).
    % Includes equilibrium point checks, Jacobi constant conservation, and randomized tests.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarthMoon = 0.01215058560962404; % Earth-Moon mass ratio
    end

    methods (Test)

        function testL1EquilibriumZeroAcceleration(testCase)
            % At L1 with zero velocity, acceleration components (excluding Coriolis) should be near zero
            dMu = testCase.dMuEarthMoon;

            % Approximate L1 position via iterative solution (Newton)
            dXL1 = testCase.computeL1(dMu);

            dStateL1 = [dXL1; 0; 0; 0; 0; 0];
            dxdt = RHS_CR3BP(dStateL1, dMu);

            % At equilibrium with zero velocity: vx=vy=vz=0, so Coriolis vanishes
            % Acceleration should be zero (within numerical precision)
            testCase.verifyLessThan(norm(dxdt(4:6)), 1e-10, ...
                'Acceleration at L1 with zero velocity should be near zero');
        end

        function testJacobiConstantConservation(testCase)
            % Jacobi constant C = -2*U(x,y,z) - (vx^2+vy^2+vz^2) must be conserved
            dMu = testCase.dMuEarthMoon;

            % Initial condition near L1 with small velocity perturbation
            dXL1 = testCase.computeL1(dMu);
            dState0 = [dXL1; 0; 0; 0; 0.01; 0];

            dJacobi0 = testCase.computeJacobiConstant(dState0, dMu);

            % Propagate for non-trivial time
            objOpts = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
            [~, dTrajectory] = ode113(@(dTime, dState) RHS_CR3BP(dState, dMu), ...
                                      linspace(0, 5, 200), dState0, objOpts);

            % Verify Jacobi constant at every output point
            for idT = 1:size(dTrajectory, 1)
                dJacobiCurrent = testCase.computeJacobiConstant(dTrajectory(idT, :)', dMu);
                dRelError = abs(dJacobiCurrent - dJacobi0) / abs(dJacobi0);
                testCase.verifyLessThan(dRelError, 1e-10, ...
                    sprintf('Jacobi constant drift at point %d exceeds tolerance', idT));
            end
        end

        function testRandomStatesJacobiConservation(testCase)
            % Randomized: verify Jacobi conservation for multiple random initial conditions
            rng('default');
            dMu = testCase.dMuEarthMoon;
            ui32NumTrials = 8;

            for idTrial = 1:ui32NumTrials
                % Random state in the vicinity of the P1-P2 system
                dState0 = [0.5 + 0.5*rand(); (rand()-0.5)*0.5; (rand()-0.5)*0.1; ...
                           (rand()-0.5)*0.2; (rand()-0.5)*0.2; (rand()-0.5)*0.05];

                dJacobi0 = testCase.computeJacobiConstant(dState0, dMu);

                objOpts = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
                [~, dTraj] = ode113(@(dTime, dState) RHS_CR3BP(dState, dMu), ...
                                    [0, 2], dState0, objOpts);

                dJacobiEnd = testCase.computeJacobiConstant(dTraj(end, :)', dMu);
                dRelError = abs(dJacobiEnd - dJacobi0) / abs(dJacobi0);

                testCase.verifyLessThan(dRelError, 1e-9, ...
                    sprintf('Jacobi drift in random trial %d', idTrial));
            end
        end

        function testSymmetryAboutXaxis(testCase)
            % CR3BP is symmetric about the x-axis: f(x,-y,-z,-vx,vy,vz) related by sign flip
            dMu = testCase.dMuEarthMoon;

            rng(7);
            for idT = 1:10
                dState = randn(6, 1) * 0.5;
                dState(1) = dState(1) + 0.8; % Offset to avoid singularity at primaries

                dxdt_orig = RHS_CR3BP(dState, dMu);

                % Mirror state about x-axis: y --> -y, z --> -z, vy --> -vy, vz --> -vz
                dStateMirror = [dState(1); -dState(2); -dState(3); dState(4); -dState(5); -dState(6)];
                dxdt_mirror = RHS_CR3BP(dStateMirror, dMu);

                % Mirrored derivatives should have same pattern of sign flips
                dExpected = [dxdt_orig(1); -dxdt_orig(2); -dxdt_orig(3); ...
                             dxdt_orig(4); -dxdt_orig(5); -dxdt_orig(6)];

                testCase.verifyEqual(dxdt_mirror, dExpected, 'AbsTol', 1e-14, ...
                    'CR3BP should be symmetric about the x-axis');
            end
        end

        function testReducesToTwoBodyAtLargeDistance(testCase)
            % Far from the secondary, the CR3BP should approximate a 2BP around the primary
            dMu = testCase.dMuEarthMoon;
            dGravP1 = 1 - dMu;

            % Place spacecraft far from secondary (at large x, near primary)
            dRadius = 50; % Far from both primaries in non-dim units
            dState = [dRadius; 0; 0; 0; 0; 0];

            dxdt = RHS_CR3BP(dState, dMu);
            dAccCR3BP = dxdt(4:6);

            % 2BP acceleration toward primary at (-mu, 0, 0) from (R, 0, 0)
            dRelPos = [dRadius + dMu; 0; 0];
            dAcc2BP = -dGravP1 / norm(dRelPos)^3 * dRelPos;
            % Add centrifugal term (x-component) in rotating frame
            dAcc2BP(1) = dAcc2BP(1) + dRadius;

            % Should be close (secondary contribution ~mu/R^2 is small)
            dRelError = norm(dAccCR3BP - dAcc2BP) / norm(dAccCR3BP);
            testCase.verifyLessThan(dRelError, 0.01, ...
                'At large distance, CR3BP should approximate 2BP within 1%');
        end

    end

    methods (Static)

        function dXL1 = computeL1(dMu)
            % Newton iteration for collinear L1 point (between P1 and P2)
            dXL1 = 1 - dMu - (dMu/3)^(1/3); % Initial guess
            for idIter = 1:50
                dD1 = dXL1 + dMu;
                dD2 = dXL1 - (1 - dMu);
                dFval = dXL1 - (1-dMu)/dD1^2*sign(dD1) + dMu/dD2^2*sign(dD2);
                dFprime = 1 + 2*(1-dMu)/abs(dD1)^3 + 2*dMu/abs(dD2)^3;
                dXL1 = dXL1 - dFval / dFprime;
            end
        end

        function dJacobi = computeJacobiConstant(dState, dMu)
            % C = 2*U - v^2, where U is the pseudo-potential
            dPosX = dState(1); dPosY = dState(2); dPosZ = dState(3);
            dVelSq = dState(4)^2 + dState(5)^2 + dState(6)^2;

            dR1 = sqrt((dPosX + dMu)^2 + dPosY^2 + dPosZ^2);
            dR2 = sqrt((dPosX - 1 + dMu)^2 + dPosY^2 + dPosZ^2);

            dPseudoPotential = 0.5*(dPosX^2 + dPosY^2) + (1-dMu)/dR1 + dMu/dR2;
            dJacobi = 2*dPseudoPotential - dVelSq;
        end

    end
end
