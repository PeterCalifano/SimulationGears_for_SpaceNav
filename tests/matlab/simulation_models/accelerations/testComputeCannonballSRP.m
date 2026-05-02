classdef testComputeCannonballSRP < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for ComputeCannonballSRP and EvalJac_CannonballSRP.
    % Covers fixed-pressure and inverse-square pressure modes, eclipse handling, and finite-difference
    % Jacobian validation.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)

        function testFixedPressureAccelerationUsesSunLineOnly(testCase)
            dPosSunToSC_IN = [3; 0; 0];
            dCoeffSRP = 2.5e-7;

            [dAccSRP_IN, dDistSunToSC, bIsSRPActive] = ComputeCannonballSRP( ...
                dPosSunToSC_IN, dCoeffSRP, false, false);

            testCase.verifyEqual(dDistSunToSC, 3.0, 'AbsTol', 0.0);
            testCase.verifyTrue(bIsSRPActive);
            testCase.verifyEqual(dAccSRP_IN, [dCoeffSRP; 0; 0], 'AbsTol', 1e-18);
        end

        function testInverseSquarePressureAccelerationMagnitude(testCase)
            dPosSunToSC_IN = [3; 0; 0];
            dCoeffSRP = 18.0;

            dAccSRP_IN = ComputeCannonballSRP(dPosSunToSC_IN, dCoeffSRP, false, true);

            testCase.verifyEqual(dAccSRP_IN, [2.0; 0; 0], 'AbsTol', 1e-14);
        end

        function testEclipseZeroesAccelerationAndJacobian(testCase)
            dPosSunToSC_IN = [2.0; -1.0; 4.0];
            dCoeffSRP = 3.0;

            [dAccSRP_IN, dDistSunToSC, bIsSRPActive] = ComputeCannonballSRP( ...
                dPosSunToSC_IN, dCoeffSRP, true, true);
            dJacSRP_IN = EvalJac_CannonballSRP( ...
                dPosSunToSC_IN, dCoeffSRP, true, true, dDistSunToSC, bIsSRPActive);

            testCase.verifyFalse(bIsSRPActive);
            testCase.verifyEqual(dAccSRP_IN, zeros(3, 1), 'AbsTol', 0.0);
            testCase.verifyEqual(dJacSRP_IN, zeros(3, 3), 'AbsTol', 0.0);
        end

        function testFixedPressureJacobianMatchesFiniteDifference(testCase)
            dPosSunToSC_IN = [2.5; -3.0; 5.0];
            dCoeffSRP = 4.0;

            dJacAnalytical = EvalJac_CannonballSRP(dPosSunToSC_IN, dCoeffSRP, false, false);
            dJacFD = testCase.finiteDifferenceJacobian(dPosSunToSC_IN, dCoeffSRP, false);

            testCase.verifyEqual(dJacAnalytical, dJacFD, 'RelTol', 1e-8, 'AbsTol', 1e-9);
        end

        function testInverseSquarePressureJacobianMatchesFiniteDifference(testCase)
            dPosSunToSC_IN = [2.5; -3.0; 5.0];
            dCoeffSRP = 18.0;

            dJacAnalytical = EvalJac_CannonballSRP(dPosSunToSC_IN, dCoeffSRP, false, true);
            dJacFD = testCase.finiteDifferenceJacobian(dPosSunToSC_IN, dCoeffSRP, true);

            testCase.verifyEqual(dJacAnalytical, dJacFD, 'RelTol', 1e-8, 'AbsTol', 1e-9);
        end

        function testCachedDistanceAndActivityMatchUncachedJacobian(testCase)
            dPosSunToSC_IN = [-7.0; 4.0; 2.0];
            dCoeffSRP = 9.0;

            [~, dDistSunToSC, bIsSRPActive] = ComputeCannonballSRP(dPosSunToSC_IN, dCoeffSRP, false, true);
            dJacCachedDistanceOnly = EvalJac_CannonballSRP( ...
                dPosSunToSC_IN, dCoeffSRP, false, true, dDistSunToSC);
            dJacCached = EvalJac_CannonballSRP( ...
                dPosSunToSC_IN, dCoeffSRP, false, true, dDistSunToSC, bIsSRPActive);
            dJacUncached = EvalJac_CannonballSRP(dPosSunToSC_IN, dCoeffSRP, false, true);

            testCase.verifyEqual(dJacCachedDistanceOnly, dJacUncached, 'AbsTol', 1e-15);
            testCase.verifyEqual(dJacCached, dJacUncached, 'AbsTol', 1e-15);
        end

        function testZeroSunSpacecraftDistanceRejected(testCase)
            testCase.verifyError(@() ComputeCannonballSRP([0; 0; 0], 1.0, false, false), ...
                'ComputeCannonballSRP:ZeroDistance');
            testCase.verifyError(@() EvalJac_CannonballSRP([0; 0; 0], 1.0, false, false), ...
                'EvalJac_CannonballSRP:ZeroDistance');
        end

    end

    methods (Access = private)

        function dJacFD = finiteDifferenceJacobian(~, dPosSunToSC_IN, dCoeffSRP, bRecomputePressureFromDistance)
            dStep = 1e-6 * norm(dPosSunToSC_IN);
            dJacFD = zeros(3, 3);

            for idxAxis = 1:3
                dPerturb = zeros(3, 1);
                dPerturb(idxAxis) = dStep;

                dAccPlus = ComputeCannonballSRP( ...
                    dPosSunToSC_IN + dPerturb, dCoeffSRP, false, bRecomputePressureFromDistance);
                dAccMinus = ComputeCannonballSRP( ...
                    dPosSunToSC_IN - dPerturb, dCoeffSRP, false, bRecomputePressureFromDistance);

                dJacFD(:, idxAxis) = (dAccPlus - dAccMinus) / (2.0 * dStep);
            end
        end

    end
end
