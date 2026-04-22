classdef testEvalJac_2BP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for EvalJac_2BP (Jacobian of the Cartesian two-body equations of motion).
    % Validates block structure, finite-difference agreement, and consistency with RHS_2BPwSTM.
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant)
        dMuEarth = 398600.4418;   % [km^3/s^2]
    end

    methods (Test)

        function testJacobianStructure(testCase)
            dState = [7000; 500; -300; -0.2; 7.3; 0.1];

            dJac = EvalJac_2BP(dState, testCase.dMuEarth);

            testCase.verifyEqual(dJac(1:3, 1:3), zeros(3), 'AbsTol', 1e-15, ...
                'Upper-left block must be zero');
            testCase.verifyEqual(dJac(1:3, 4:6), eye(3), 'AbsTol', 1e-15, ...
                'Upper-right block must be identity');
            testCase.verifyEqual(dJac(4:6, 4:6), zeros(3), 'AbsTol', 1e-15, ...
                'Lower-right block must be zero');
            testCase.verifyEqual(dJac(4:6, 1:3), dJac(4:6, 1:3)', 'AbsTol', 1e-12, ...
                'Gravity-gradient block must be symmetric');
        end

        function testFiniteDifferenceAccuracy(testCase)
            dState = [7200; -450; 220; 0.4; 7.1; -0.2];
            dEpsilon = 1e-6;
            strDynParams.dGravParam = testCase.dMuEarth;

            dJacAnalytical = EvalJac_2BP(dState, testCase.dMuEarth);
            dJacNumerical = zeros(6, 6);

            for idCol = 1:6
                dStatePlus = dState;
                dStateMinus = dState;
                dStatePlus(idCol) = dStatePlus(idCol) + dEpsilon;
                dStateMinus(idCol) = dStateMinus(idCol) - dEpsilon;

                dFplus = RHS_2BP(0, dStatePlus, strDynParams);
                dFminus = RHS_2BP(0, dStateMinus, strDynParams);
                dJacNumerical(:, idCol) = (dFplus - dFminus) / (2 * dEpsilon);
            end

            dRelError = norm(dJacAnalytical - dJacNumerical, 'fro') / norm(dJacNumerical, 'fro');
            testCase.verifyLessThan(dRelError, 1e-7, ...
                'Analytical Jacobian must match finite differences of RHS_2BP');
        end

        function testConsistencyWithRHS2BPwSTM(testCase)
            dState = [7600; 200; -150; -0.1; 7.15; 0.08; reshape(eye(6), 36, 1)];

            dxPhidt = RHS_2BPwSTM(0, dState, testCase.dMuEarth);
            dJacFromSTM = reshape(dxPhidt(7:end), 6, 6);
            dJacDirect = EvalJac_2BP(dState(1:6), testCase.dMuEarth);

            testCase.verifyEqual(dJacFromSTM, dJacDirect, 'AbsTol', 1e-14, ...
                'At Phi=I, RHS_2BPwSTM must expose the same Jacobian as EvalJac_2BP');
        end

    end
end
