classdef testEvalRHS_InertialDynOrbitSRP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Focused tests for cannonball SRP inside evalRHS_InertialDynOrbit.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)

        function testFixedPressureSRPContribution(testCase)
            dCoeffSRP = 4.0e-7;

            [dxdt, strAccelInfo] = testCase.evaluateSRPOnlyRHS(dCoeffSRP, false);

            testCase.verifyEqual(dxdt(4:6), [dCoeffSRP; 0; 0], 'AbsTol', 1e-18);
            testCase.verifyEqual(strAccelInfo.dAccCannonBallSRP, [dCoeffSRP; 0; 0], 'AbsTol', 1e-18);
            testCase.verifyEqual(strAccelInfo.dSRPdistToSun, 3.0, 'AbsTol', 0.0);
            testCase.verifyTrue(strAccelInfo.bIsSRPActive);
        end

        function testEclipseSuppressesSRPContribution(testCase)
            dCoeffSRP = 18.0;

            [dxdt, strAccelInfo] = testCase.evaluateSRPOnlyRHS(dCoeffSRP, true);

            testCase.verifyEqual(dxdt(4:6), zeros(3, 1), 'AbsTol', 0.0);
            testCase.verifyEqual(strAccelInfo.dAccCannonBallSRP, zeros(3, 1), 'AbsTol', 0.0);
            testCase.verifyFalse(strAccelInfo.bIsSRPActive);
        end

    end

    methods (Access = private)

        function [dxdt, strAccelInfo] = evaluateSRPOnlyRHS(~, dCoeffSRP, bIsInEclipse)
            dxState_IN = [4; 0; 0; 0; 0; 0];
            dSunPos_IN = [1; 0; 0];

            [dxdt, strAccelInfo] = evalRHS_InertialDynOrbit( ...
                dxState_IN, ...
                zeros(3, 3), ...
                0.0, ...
                1.0, ...
                dCoeffSRP, ...
                0.0, ...
                dSunPos_IN, ...
                [], ...
                uint32(0), ...
                [], ...
                zeros(3, 1), ...
                bIsInEclipse);
        end

    end
end
