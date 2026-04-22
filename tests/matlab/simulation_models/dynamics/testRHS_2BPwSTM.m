classdef testRHS_2BPwSTM < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for RHS_2BPwSTM (two-body problem ODE with State Transition Matrix propagation).
    % Validates state dynamics, Jacobian structure, STM volume-preservation, and energy conservation.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth = 398600.4418;   % [km^3/s^2]
    end

    methods (Test)

        function testOutputLength(testCase)
            % Output must be [42x1]: 6 state + 36 STM elements
            dState = [7000; 0; 0; 0; 7.5; 0; reshape(eye(6), 36, 1)];
            dxPhidt = RHS_2BPwSTM(0, dState, testCase.dMuEarth);
            testCase.verifySize(dxPhidt, [42, 1]);
        end

        function testStateMatchesTwoBodyDynamics(testCase)
            % First 6 elements must satisfy the Keplerian equations of motion
            dPosVel = [7000; 500; -300; -0.2; 7.3; 0.1];
            dState  = [dPosVel; reshape(eye(6), 36, 1)];
            dMu     = testCase.dMuEarth;

            dxPhidt = RHS_2BPwSTM(0, dState, dMu);

            % Velocity portion
            testCase.verifyEqual(dxPhidt(1:3), dPosVel(4:6), 'AbsTol', 1e-14, ...
                'Velocity components must pass through unchanged');

            % Acceleration: -mu/r^3 * r
            dR = norm(dPosVel(1:3));
            dExpAccel = -dMu / dR^3 * dPosVel(1:3);
            testCase.verifyEqual(dxPhidt(4:6), dExpAccel, 'AbsTol', 1e-10, ...
                'Acceleration must match -mu/r^3 * r');
        end

        function testJacobianBlockStructure(testCase)
            % At Phi=I: dPhidt = A*I = A. Verify known block structure of A.
            % Upper-left 3x3: zeros (position does not drive itself)
            % Upper-right 3x3: identity (velocity drives position)
            % Lower-right 3x3: zeros (velocity does not drive itself in 2BP)
            % Lower-left 3x3: gravity gradient (symmetric tensor)
            dMu = testCase.dMuEarth;
            rng(42);

            for idTrial = 1:8
                dPos = randn(3, 1) * 500 + [7000; 0; 0];
                dVel = randn(3, 1) * 0.5;
                dState = [dPos; dVel; reshape(eye(6), 36, 1)];

                dxPhidt = RHS_2BPwSTM(0, dState, dMu);
                dA = reshape(dxPhidt(7:end), 6, 6);

                testCase.verifyEqual(dA(1:3, 1:3), zeros(3), 'AbsTol', 1e-14, ...
                    sprintf('Upper-left block must be zero (trial %d)', idTrial));
                testCase.verifyEqual(dA(1:3, 4:6), eye(3), 'AbsTol', 1e-14, ...
                    sprintf('Upper-right block must be identity (trial %d)', idTrial));
                testCase.verifyEqual(dA(4:6, 4:6), zeros(3), 'AbsTol', 1e-14, ...
                    sprintf('Lower-right block must be zero (trial %d)', idTrial));

                % Gravity gradient must be symmetric
                dGravGrad = dA(4:6, 1:3);
                testCase.verifyEqual(dGravGrad, dGravGrad', 'AbsTol', 1e-12, ...
                    sprintf('Gravity gradient Jacobian must be symmetric (trial %d)', idTrial));
            end
        end

        function testJacobianMatchesFiniteDifferences(testCase)
            % Jacobian extracted from STM derivative must match FD of the state dynamics
            dMu  = testCase.dMuEarth;
            dEps = 1e-6;
            rng('default');

            for idTrial = 1:10
                dPos = randn(3, 1) * 800 + [7000; 0; 0];
                dVel = randn(3, 1) * 0.4;
                dStateBase = [dPos; dVel; reshape(eye(6), 36, 1)];

                % Finite-difference Jacobian of first 6 outputs w.r.t. first 6 inputs
                dJacFD = zeros(6, 6);
                for idCol = 1:6
                    dSp = dStateBase; dSp(idCol) = dSp(idCol) + dEps;
                    dSm = dStateBase; dSm(idCol) = dSm(idCol) - dEps;
                    dFp = RHS_2BPwSTM(0, dSp, dMu);
                    dFm = RHS_2BPwSTM(0, dSm, dMu);
                    dJacFD(:, idCol) = (dFp(1:6) - dFm(1:6)) / (2 * dEps);
                end

                % Analytical Jacobian: at Phi=I, dPhidt = A*I = A
                dxPhiI = RHS_2BPwSTM(0, dStateBase, dMu);
                dJacAnalytical = reshape(dxPhiI(7:end), 6, 6);

                dRelErr = norm(dJacAnalytical - dJacFD, 'fro') / (norm(dJacFD, 'fro') + eps);
                testCase.verifyLessThan(dRelErr, 1e-4, ...
                    sprintf('Jacobian FD mismatch at trial %d (relErr=%.2e)', idTrial, dRelErr));
            end
        end

        function testSTMDeterminantEqualsOne(testCase)
            % Propagating the coupled state+STM ODE must preserve det(Phi)=1 (volume preservation)
            dMu = testCase.dMuEarth;

            rng(7);
            for idTrial = 1:3
                dSma = 7000 + 3000 * rand();
                dEcc = 0.05 + 0.1 * rand();
                dVp  = sqrt(dMu / dSma * (1 + dEcc) / (1 - dEcc));  % velocity at periapsis
                dPosVel0 = [dSma * (1 - dEcc); 0; 0; 0; dVp; 0];
                dState0  = [dPosVel0; reshape(eye(6), 36, 1)];

                objOpts = odeset('RelTol', 1e-12, 'AbsTol', 1e-13);
                [~, dTraj] = ode113(@(dT, dS) RHS_2BPwSTM(dT, dS, dMu), ...
                    linspace(0, 2500, 80), dState0, objOpts);

                for idT = 1:size(dTraj, 1)
                    dPhi    = reshape(dTraj(idT, 7:end), 6, 6);
                    dDetPhi = det(dPhi);
                    testCase.verifyEqual(dDetPhi, 1.0, 'AbsTol', 1e-5, ...
                        sprintf('det(STM) must be 1 at trial %d step %d (got %.6f)', idTrial, idT, dDetPhi));
                end
            end
        end

        function testEnergyConservationDuringPropagation(testCase)
            % Orbital energy must be conserved during 2BPwSTM propagation
            % (state dynamics must be the same as 2BP regardless of STM)
            dMu = testCase.dMuEarth;

            rng('default');
            for idTrial = 1:5
                dSma = 7000 + 4000 * rand();
                dEcc = 0.2 * rand();
                dVp  = sqrt(dMu / dSma * (1 + dEcc) / (1 - dEcc));
                dPosVel0 = [dSma * (1 - dEcc); 0; 0; 0; dVp; 0];
                dState0  = [dPosVel0; reshape(eye(6), 36, 1)];

                dE0 = 0.5 * norm(dPosVel0(4:6))^2 - dMu / norm(dPosVel0(1:3));

                objOpts = odeset('RelTol', 1e-12, 'AbsTol', 1e-13);
                [~, dTraj] = ode113(@(dT, dS) RHS_2BPwSTM(dT, dS, dMu), ...
                    linspace(0, 3000, 60), dState0, objOpts);

                for idT = 1:size(dTraj, 1)
                    dPV = dTraj(idT, 1:6)';
                    dE  = 0.5 * norm(dPV(4:6))^2 - dMu / norm(dPV(1:3));
                    testCase.verifyEqual(dE, dE0, 'RelTol', 1e-8, ...
                        sprintf('Energy drift at trial %d step %d', idTrial, idT));
                end
            end
        end

        function testSTMSatisfiesVariationalEquation(testCase)
            % Verify that dPhidt = A(t) * Phi(t) holds during propagation
            % by checking consistency between the propagated STM derivative and A*Phi
            dMu = testCase.dMuEarth;
            dPosVel0 = [7500; 200; -100; -0.1; 7.4; 0.05];
            dState0  = [dPosVel0; reshape(eye(6), 36, 1)];

            objOpts = odeset('RelTol', 1e-12, 'AbsTol', 1e-13);
            dTimes = linspace(0, 1000, 20);
            [~, dTraj] = ode113(@(dT, dS) RHS_2BPwSTM(dT, dS, dMu), dTimes, dState0, objOpts);

            for idT = 1:size(dTraj, 1)
                dStateCurr = dTraj(idT, :)';
                dPhi       = reshape(dStateCurr(7:end), 6, 6);

                % Get derivatives from RHS
                dxPhidt = RHS_2BPwSTM(0, dStateCurr, dMu);
                dPhidtRHS = reshape(dxPhidt(7:end), 6, 6);

                % Extract A at current state (use Phi=I trick)
                dStateForA = [dStateCurr(1:6); reshape(eye(6), 36, 1)];
                dxA = RHS_2BPwSTM(0, dStateForA, dMu);
                dA  = reshape(dxA(7:end), 6, 6);

                % Variational equation: dPhidt = A * Phi
                dPhidtVariational = dA * dPhi;

                dRelErr = norm(dPhidtRHS - dPhidtVariational, 'fro') / (norm(dPhidtVariational, 'fro') + eps);
                testCase.verifyLessThan(dRelErr, 1e-10, ...
                    sprintf('Variational equation dPhidt = A*Phi violated at step %d', idT));
            end
        end

    end
end
