classdef testRHS_CR3BPwSTM < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for RHS_CR3BPwSTM (CR3BP with State Transition Matrix propagation).
    % Validates STM properties: initial identity, determinant, Jacobi conservation of underlying state.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarthMoon = 0.01215058560962404;
    end

    methods (Test)

        function testStateDynamicsMatchRHS_CR3BP(testCase)
            % First 6 components of RHS_CR3BPwSTM must equal RHS_CR3BP
            dMu = testCase.dMuEarthMoon;
            dState6 = [0.8; 0.15; 0.05; 0.01; -0.02; 0.005];
            dSTM0 = reshape(eye(6), 36, 1);
            dFullState = [dState6; dSTM0];

            dxdt_full = RHS_CR3BPwSTM(0, dFullState, dMu);
            dxdt_cr3bp = RHS_CR3BP(dState6, dMu);

            testCase.verifyEqual(dxdt_full(1:6), dxdt_cr3bp, 'AbsTol', 1e-14, ...
                'State dynamics must match standalone RHS_CR3BP');
        end

        function testSTMStartsAsIdentity(testCase)
            % With STM initialized as identity, after short propagation det(STM) ~ 1 (symplectic)
            dMu = testCase.dMuEarthMoon;
            dState6 = [0.8; 0.1; 0.05; 0.01; -0.02; 0.005];
            dSTM0 = reshape(eye(6), 36, 1);
            dFullState0 = [dState6; dSTM0];

            objOpts = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
            [~, dTraj] = ode113(@(dTime, dState) RHS_CR3BPwSTM(dTime, dState, dMu), ...
                                linspace(0, 2, 50), dFullState0, objOpts);

            % At t=0, STM should be identity
            dSTM_initial = reshape(dTraj(1, 7:42), 6, 6);
            testCase.verifyEqual(dSTM_initial, eye(6), 'AbsTol', 1e-14);

            % Check determinant of STM remains close to 1 (symplecticity)
            for idT = 1:size(dTraj, 1)
                dSTM = reshape(dTraj(idT, 7:42), 6, 6);
                dDetSTM = det(dSTM);
                testCase.verifyEqual(dDetSTM, 1, 'AbsTol', 1e-6, ...
                    sprintf('STM determinant deviates from 1 at point %d: %.6e', idT, dDetSTM));
            end
        end

        function testJacobiConservationOfOrbitalState(testCase)
            % Underlying orbital state must conserve Jacobi constant
            dMu = testCase.dMuEarthMoon;
            dState6 = [0.8; 0.1; 0.05; 0.01; -0.02; 0.005];
            dSTM0 = reshape(eye(6), 36, 1);
            dFullState0 = [dState6; dSTM0];

            dJacobi0 = testRHS_CR3BP.computeJacobiConstant(dState6, dMu);

            objOpts = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
            [~, dTraj] = ode113(@(dTime, dState) RHS_CR3BPwSTM(dTime, dState, dMu), ...
                                linspace(0, 3, 100), dFullState0, objOpts);

            for idT = 1:size(dTraj, 1)
                dJacobiCurr = testRHS_CR3BP.computeJacobiConstant(dTraj(idT, 1:6)', dMu);
                dRelErr = abs(dJacobiCurr - dJacobi0) / abs(dJacobi0);
                testCase.verifyLessThan(dRelErr, 1e-9);
            end
        end

        function testRandomInitialConditions(testCase)
            % Randomized: STM determinant must remain ~1 for random ICs
            rng('default');
            dMu = testCase.dMuEarthMoon;

            for idTrial = 1:8
                dState6 = randn(6, 1) * 0.2;
                dState6(1) = dState6(1) + 0.5 + rand();
                dSTM0 = reshape(eye(6), 36, 1);
                dFullState0 = [dState6; dSTM0];

                objOpts = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
                [~, dTraj] = ode113(@(dTime, dState) RHS_CR3BPwSTM(dTime, dState, dMu), ...
                                    [0, 1], dFullState0, objOpts);

                dSTMfinal = reshape(dTraj(end, 7:42), 6, 6);
                testCase.verifyEqual(det(dSTMfinal), 1, 'AbsTol', 1e-4, ...
                    sprintf('STM determinant off at trial %d', idTrial));
            end
        end

    end
end
