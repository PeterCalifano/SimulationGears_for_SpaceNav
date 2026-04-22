classdef testEvalJac_CR3BP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for EvalJac_CR3BP (Jacobian of the CR3BP equations of motion).
    % Validates against finite differences, checks structural properties, and uses randomized inputs.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarthMoon = 0.01215058560962404;
    end

    methods (Test)

        function testJacobianStructure(testCase)
            % Verify the Jacobian has correct block structure:
            % A = [0 I; Uxx C] where upper-left is 3x3 zeros, upper-right is 3x3 identity
            dMu = testCase.dMuEarthMoon;
            dState = [0.8; 0.1; 0.05; 0; 0; 0];

            dJac = EvalJac_CR3BP(dState, dMu);

            % Upper-left 3x3 should be zeros
            testCase.verifyEqual(dJac(1:3, 1:3), zeros(3), 'AbsTol', 1e-15, ...
                'Upper-left block must be zeros');

            % Upper-right 3x3 should be identity
            testCase.verifyEqual(dJac(1:3, 4:6), eye(3), 'AbsTol', 1e-15, ...
                'Upper-right block must be identity');

            % Coriolis terms: A(4,5)=2, A(5,4)=-2, A(6,6)=0
            testCase.verifyEqual(dJac(4, 5), 2, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(5, 4), -2, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(4, 6), 0, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(5, 6), 0, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(6, 4), 0, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJac(6, 5), 0, 'AbsTol', 1e-15);
        end

        function testHessianSymmetry(testCase)
            % The position Hessian (Uxx, lower-left 3x3 minus centrifugal/Coriolis) must be symmetric
            dMu = testCase.dMuEarthMoon;

            rng(42);
            for idT = 1:10
                dState = randn(6, 1) * 0.3;
                dState(1) = dState(1) + 0.8; % Avoid primaries

                dJac = EvalJac_CR3BP(dState, dMu);

                % Extract Uxx from lower-left (subtract centrifugal diagonal)
                dUxxBlock = dJac(4:6, 1:3);
                dUxxBlock(1,1) = dUxxBlock(1,1) - 1; % Remove centrifugal x
                dUxxBlock(2,2) = dUxxBlock(2,2) - 1; % Remove centrifugal y

                testCase.verifyEqual(dUxxBlock, dUxxBlock', 'AbsTol', 1e-14, ...
                    'Hessian of pseudo-potential must be symmetric');
            end
        end

        function testFiniteDifferenceAccuracy(testCase)
            % Compare analytical Jacobian against central finite differences of RHS_CR3BP
            dMu = testCase.dMuEarthMoon;
            dState = [0.8; 0.15; 0.05; 0.01; -0.02; 0.005];
            dEpsilon = 1e-7;

            dJacAnalytical = EvalJac_CR3BP(dState, dMu);
            dJacNumerical = zeros(6, 6);

            for idCol = 1:6
                dStatePlus  = dState;
                dStateMinus = dState;
                dStatePlus(idCol)  = dStatePlus(idCol) + dEpsilon;
                dStateMinus(idCol) = dStateMinus(idCol) - dEpsilon;

                dFplus  = RHS_CR3BP(dStatePlus, dMu);
                dFminus = RHS_CR3BP(dStateMinus, dMu);
                dJacNumerical(:, idCol) = (dFplus - dFminus) / (2 * dEpsilon);
            end

            dRelError = norm(dJacAnalytical - dJacNumerical, 'fro') / norm(dJacNumerical, 'fro');
            testCase.verifyLessThan(dRelError, 1e-6, ...
                'Analytical Jacobian must match finite-difference within 1e-6 relative');
        end

        function testRandomStatesFiniteDifference(testCase)
            % Randomized finite-difference validation at multiple states
            dMu = testCase.dMuEarthMoon;
            dEpsilon = 1e-7;

            rng('default');
            ui32NumTrials = 15;

            for idTrial = 1:ui32NumTrials
                dState = randn(6, 1) * 0.3;
                dState(1) = dState(1) + 0.5 + rand(); % Keep away from primaries

                dJacAnalytical = EvalJac_CR3BP(dState, dMu);
                dJacNumerical = zeros(6, 6);

                for idCol = 1:6
                    dStatePlus  = dState;
                    dStateMinus = dState;
                    dStatePlus(idCol)  = dStatePlus(idCol) + dEpsilon;
                    dStateMinus(idCol) = dStateMinus(idCol) - dEpsilon;

                    dFplus  = RHS_CR3BP(dStatePlus, dMu);
                    dFminus = RHS_CR3BP(dStateMinus, dMu);
                    dJacNumerical(:, idCol) = (dFplus - dFminus) / (2 * dEpsilon);
                end

                dRelError = norm(dJacAnalytical - dJacNumerical, 'fro') / norm(dJacNumerical, 'fro');
                testCase.verifyLessThan(dRelError, 1e-6, ...
                    sprintf('Jacobian FD mismatch at random trial %d', idTrial));
            end
        end

        function testDifferentMassRatios(testCase)
            % Verify Jacobian works for different mass ratios
            dState = [0.8; 0.1; 0.05; 0; 0; 0];
            dEpsilon = 1e-7;

            dMassRatios = [0.001, 0.01, 0.1, 0.3, 0.5];
            for idMu = 1:length(dMassRatios)
                dMu = dMassRatios(idMu);
                dJacAnalytical = EvalJac_CR3BP(dState, dMu);
                dJacNumerical = zeros(6, 6);

                for idCol = 1:6
                    dStatePlus  = dState; dStatePlus(idCol)  = dStatePlus(idCol) + dEpsilon;
                    dStateMinus = dState; dStateMinus(idCol) = dStateMinus(idCol) - dEpsilon;
                    dJacNumerical(:, idCol) = (RHS_CR3BP(dStatePlus, dMu) - RHS_CR3BP(dStateMinus, dMu)) / (2*dEpsilon);
                end

                dRelError = norm(dJacAnalytical - dJacNumerical, 'fro') / norm(dJacNumerical, 'fro');
                testCase.verifyLessThan(dRelError, 1e-6, ...
                    sprintf('Jacobian FD mismatch for mu=%.3f', dMu));
            end
        end

    end
end
