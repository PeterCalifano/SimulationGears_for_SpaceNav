classdef testFiniteBurnThrustModel < matlab.unittest.TestCase
    methods (Test)
        function testZeroDeltaVProducesInactiveBurn(testCase)
            strBurnData = ComputeFiniteBurnFromDeltaV(zeros(3,1), 500.0, 0.0, 220.0);

            testCase.verifyEqual(strBurnData.dBurnDuration, 0.0, 'AbsTol', 0.0);
            testCase.verifyEqual(strBurnData.dPropellantMass, 0.0, 'AbsTol', 0.0);
            testCase.verifyEqual(strBurnData.dBurnDirection_IN, zeros(3,1), 'AbsTol', 0.0);

            [dAccel_IN, dMassKg, bIsActive] = EvalFiniteBurnAccel(10.0, strBurnData);
            testCase.verifyFalse(bIsActive);
            testCase.verifyEqual(dAccel_IN, zeros(3,1), 'AbsTol', 0.0);
            testCase.verifyEqual(dMassKg, strBurnData.dInitialMass, 'AbsTol', 0.0);
        end

        function testNonzeroDeltaVRequiresPositiveThrust(testCase)
            dDeltaV_IN = [0.0; 0.012; 0.0];

            testCase.verifyError(@() ComputeFiniteBurnFromDeltaV(dDeltaV_IN, 500.0, 0.0, 220.0), ...
                                 'ComputeFiniteBurnFromDeltaV:InvalidThrust');
        end

        function testRocketEquationMassBudgetAndDuration(testCase)
            dDeltaV_IN = [0.0; 0.012; 0.0];
            dInitialMass = 500.0;
            dThrust = 0.25;
            dIsp = 230.0;
            dG0 = 9.80665;

            strBurnData = ComputeFiniteBurnFromDeltaV(dDeltaV_IN, dInitialMass, dThrust, dIsp);

            dExpectedFinalMass = dInitialMass * exp(-(1000.0 * norm(dDeltaV_IN)) / (dIsp * dG0));
            dExpectedPropMass = dInitialMass - dExpectedFinalMass;
            dExpectedMassFlowRate = dThrust / (dIsp * dG0);
            dExpectedBurnDuration = dExpectedPropMass / dExpectedMassFlowRate;

            testCase.verifyEqual(strBurnData.dFinalMass, dExpectedFinalMass, 'RelTol', 1.0e-14);
            testCase.verifyEqual(strBurnData.dPropellantMass, dExpectedPropMass, 'RelTol', 1.0e-14);
            testCase.verifyEqual(strBurnData.dMassFlowRate, dExpectedMassFlowRate, 'RelTol', 1.0e-14);
            testCase.verifyEqual(strBurnData.dBurnDuration, dExpectedBurnDuration, 'RelTol', 1.0e-14);
            testCase.verifyEqual(strBurnData.dBurnDirection_IN, [0.0; 1.0; 0.0], 'AbsTol', 0.0);
        end

        function testFiniteBurnAccelerationUsesCurrentMass(testCase)
            dStartTime = 120.0;
            strBurnData = ComputeFiniteBurnFromDeltaV([0.0; 0.012; 0.0], ...
                                                       500.0, ...
                                                       0.25, ...
                                                       230.0, ...
                                                       'dStartTime', dStartTime);
            dMidTime = dStartTime + 0.5 * strBurnData.dBurnDuration;

            [dAccelStart_IN, dMassStartKg, bStartIsActive] = EvalFiniteBurnAccel(dStartTime, strBurnData);
            [dAccelMid_IN, dMassMidKg, bMidIsActive] = EvalFiniteBurnAccel(dMidTime, strBurnData);
            [dAccelEnd_IN, dMassEndKg, bEndIsActive] = EvalFiniteBurnAccel(strBurnData.dEndTime, strBurnData);

            dExpectedMidMass = strBurnData.dInitialMass - ...
                strBurnData.dMassFlowRate * (dMidTime - dStartTime);

            testCase.verifyTrue(bStartIsActive);
            testCase.verifyTrue(bMidIsActive);
            testCase.verifyFalse(bEndIsActive);
            testCase.verifyEqual(dMassStartKg, strBurnData.dInitialMass, 'AbsTol', 1.0e-14);
            testCase.verifyEqual(dMassMidKg, dExpectedMidMass, 'AbsTol', 1.0e-14);
            testCase.verifyEqual(dMassEndKg, strBurnData.dFinalMass, 'AbsTol', 1.0e-14);
            testCase.verifyEqual(norm(dAccelStart_IN), strBurnData.dThrust / strBurnData.dInitialMass / 1000.0, ...
                                 'RelTol', 1.0e-14);
            testCase.verifyEqual(norm(dAccelMid_IN), strBurnData.dThrust / dExpectedMidMass / 1000.0, ...
                                 'RelTol', 1.0e-14);
            testCase.verifyEqual(dAccelEnd_IN, zeros(3,1), 'AbsTol', 0.0);
        end

        function testMeterLengthUnitUsesMeterDeltaVAndAcceleration(testCase)
            dDeltaV_kmps = [0.0; 0.012; 0.0];
            dDeltaV_mps = 1000.0 * dDeltaV_kmps;

            strBurnKm = ComputeFiniteBurnFromDeltaV(dDeltaV_kmps, ...
                                                    500.0, ...
                                                    0.25, ...
                                                    230.0, ...
                                                    'charLengthUnits', EnumLengthUnits.km);
            strBurnM = ComputeFiniteBurnFromDeltaV(dDeltaV_mps, ...
                                                   500.0, ...
                                                   0.25, ...
                                                   230.0, ...
                                                   'charLengthUnits', EnumLengthUnits.m);

            [dAccelKm_IN, dMassKm, bKmIsActive] = EvalFiniteBurnAccel(strBurnKm.dStartTime, strBurnKm);
            [dAccelM_IN, dMassM, bMIsActive] = EvalFiniteBurnAccel(strBurnM.dStartTime, strBurnM);

            testCase.verifyEqual(strBurnM.charLengthUnits, 'm');
            testCase.verifyEqual(strBurnM.dLengthUnitInMeters, 1.0, 'AbsTol', 0.0);
            testCase.verifyEqual(strBurnM.dDeltaVNorm, norm(dDeltaV_mps), 'AbsTol', 0.0);
            testCase.verifyEqual(strBurnM.dBurnDuration, strBurnKm.dBurnDuration, 'RelTol', 1.0e-14);
            testCase.verifyEqual(strBurnM.dFinalMass, strBurnKm.dFinalMass, 'RelTol', 1.0e-14);
            testCase.verifyEqual(dMassM, dMassKm, 'AbsTol', 0.0);
            testCase.verifyTrue(bKmIsActive);
            testCase.verifyTrue(bMIsActive);
            testCase.verifyEqual(dAccelM_IN, 1000.0 * dAccelKm_IN, 'RelTol', 1.0e-14);
        end
    end
end
