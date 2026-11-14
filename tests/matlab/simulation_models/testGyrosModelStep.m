classdef testGyrosModelStep < matlab.unittest.TestCase
%% DESCRIPTION
% GyrosModelStep, the gyroscope model behind GyrosModel: deterministic rate path (scale factor, misalignment,
% mounting), draw order and parameter mapping by replay of the global stream (RRW three values, then ARW three
% values, each scaled by sqrt(variance / (2 Ts))), bias stepped before use, zero variance drawing nothing, and a
% guard on the name resolution that motivates GyrosModelStep: GyrosModel resolves to the Simulink model in the
% sensors folder (a MATLAB call reaches GyrosModel.slx, not GyrosModel.m), GyrosModelStep to its .m file.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-10-2026  Pietro Califano, Claude Opus 5.5  First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SetupSimGears, GyrosModelStep; Simulink (name resolution of GyrosModel.slx).
% -------------------------------------------------------------------------------------------------------------

properties
    charOriginalPath
    objOriginalGlobalStream
end

properties (Constant)
    dSamplingTime_s = 0.01
    % Random-walk coefficients N (ARW) and K (RRW) of the MEMS preset of the image-motion recordings; the model
    % variances are 2 N^2 and 2 K^2 Ts^2
    dAngleRandomWalk_rad_per_sqrt_s = 0.2 * pi / 180 / 60
    dRateRandomWalk_rad_per_s_per_sqrt_s = 3 * pi / 180 / 3600 / 60
end

methods (TestClassSetup)
    function addSimulationGearsPaths(testCase)
        testCase.charOriginalPath = path;
        charRepoRoot = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..');
        addpath(fullfile(charRepoRoot, 'matlab'));
        SetupSimGears;
        testCase.addTeardown(@path, testCase.charOriginalPath);
    end
end

methods (TestMethodSetup)
    function keepGlobalStream(testCase)
        testCase.objOriginalGlobalStream = RandStream.getGlobalStream();
        testCase.addTeardown(@RandStream.setGlobalStream, testCase.objOriginalGlobalStream);
    end
end

methods (Test)
    function testNameResolutionOfGyrosModel(testCase)
        % GyrosModel.slx shadows GyrosModel.m in the same folder: which lists the model first and a MATLAB call
        % reaches the Simulink model, which rejects the arguments. exist reports 2 for this name on R2024b (a .m
        % file of that name exists), so it cannot show the shadowing. GyrosModelStep is the MATLAB entry point.
        charSensorsFolder = fileparts(which('GyrosModelStep'));
        testCase.verifyEqual(which('GyrosModel'), fullfile(charSensorsFolder, 'GyrosModel.slx'));
        testCase.verifyEqual(which('GyrosModel', '-all'), {fullfile(charSensorsFolder, 'GyrosModel.slx'); ...
            fullfile(charSensorsFolder, 'GyrosModel.m')});
        testCase.verifyError(@() GyrosModel(zeros(3, 1), zeros(3, 1), eye(3), eye(3), 0, 0, zeros(3, 1), ...
            testCase.dSamplingTime_s), 'Simulink:Engine:InvalidNumModelArg');
        testCase.verifyTrue(endsWith(which('GyrosModelStep'), fullfile('simulink', 'sensors', 'GyrosModelStep.m')));
    end

    function testDeterministicRatePath(testCase)
        % Zero variances: omega_m = (I + diag(s)) M R omega + b_prev.
        dScaleFactor = [1e-3; -2e-3; 5e-4];
        dMisalignment = [1, 1e-3, -2e-3; 0, 1, 4e-3; 0, 0, 1];
        dMount_IMUfromSCB = AxisAngleToDCM_([1; 2; 3], 0.4);
        dTrueRate = [0.01; -0.02; 0.03];
        dPreviousBias = [1e-5; -2e-5; 3e-5];

        [dMeasured, dBias] = GyrosModelStep(dTrueRate, dScaleFactor, dMisalignment, dMount_IMUfromSCB, 0, 0, ...
            dPreviousBias, testCase.dSamplingTime_s);

        testCase.verifyEqual(dBias, dPreviousBias);
        testCase.verifyEqual(dMeasured, (eye(3) + diag(dScaleFactor)) * dMisalignment * dMount_IMUfromSCB * ...
            dTrueRate + dPreviousBias, 'AbsTol', 1e-15);
    end

    function testDrawOrderAndParameterMappingReplay(testCase)
        % With variances 2 K^2 Ts^2 (RRW) and 2 N^2 (ARW), the increments are K sqrt(Ts) and N / sqrt(Ts) times
        % standard normal draws, RRW first, then ARW, from the global stream.
        dTs = testCase.dSamplingTime_s;
        dK = testCase.dRateRandomWalk_rad_per_s_per_sqrt_s;
        dN = testCase.dAngleRandomWalk_rad_per_sqrt_s;
        dMount_IMUfromSCB = AxisAngleToDCM_([0; 1; 1], -0.7);
        dTrueRate = [0.002; 0.001; -0.004];
        dPreviousBias = [2e-5; 1e-5; -3e-5];

        RandStream.setGlobalStream(RandStream('mrg32k3a', 'Seed', 2047));
        [dMeasured, dBias] = GyrosModelStep(dTrueRate, zeros(3, 1), eye(3), dMount_IMUfromSCB, 2 * dK^2 * dTs^2, ...
            2 * dN^2, dPreviousBias, dTs);
        RandStream.setGlobalStream(RandStream('mrg32k3a', 'Seed', 2047));
        dRrwDraws = randn(3, 1);
        dArwDraws = randn(3, 1);

        dExpectedBias = dPreviousBias + dK * sqrt(dTs) * dRrwDraws;
        testCase.verifyEqual(dBias, dExpectedBias, 'RelTol', 1e-12);
        testCase.verifyEqual(dMeasured, dMount_IMUfromSCB * dTrueRate + dExpectedBias + dN / sqrt(dTs) * dArwDraws, ...
            'RelTol', 1e-12);
    end

    function testBiasIsSteppedBeforeUse(testCase)
        % Without white noise, the first output already carries one bias increment: omega_m - R omega = b_k
        % with b_k ~= b_{k-1}.
        dTs = testCase.dSamplingTime_s;
        dK = testCase.dRateRandomWalk_rad_per_s_per_sqrt_s;
        dTrueRate = [0.01; 0.02; -0.01];
        dPreviousBias = zeros(3, 1);

        RandStream.setGlobalStream(RandStream('mrg32k3a', 'Seed', 11));
        [dMeasured, dBias] = GyrosModelStep(dTrueRate, zeros(3, 1), eye(3), eye(3), 2 * dK^2 * dTs^2, 0, ...
            dPreviousBias, dTs);

        testCase.verifyTrue(all(dBias ~= dPreviousBias));
        testCase.verifyEqual(dMeasured - dTrueRate, dBias, 'AbsTol', 1e-16);
    end

    function testZeroVarianceDrawsNothing(testCase)
        % RRW variance 0: the ARW increments are the first three draws of the stream. Both variances 0: the
        % stream state does not change.
        dTs = testCase.dSamplingTime_s;
        dN = testCase.dAngleRandomWalk_rad_per_sqrt_s;
        objStream = RandStream('mrg32k3a', 'Seed', 5);
        RandStream.setGlobalStream(objStream);
        dMeasured = GyrosModelStep(zeros(3, 1), zeros(3, 1), eye(3), eye(3), 0, 2 * dN^2, zeros(3, 1), dTs);

        RandStream.setGlobalStream(RandStream('mrg32k3a', 'Seed', 5));
        testCase.verifyEqual(dMeasured, dN / sqrt(dTs) * randn(3, 1), 'RelTol', 1e-12);

        objStream = RandStream('mrg32k3a', 'Seed', 6);
        RandStream.setGlobalStream(objStream);
        ui32StateBefore = objStream.State;
        GyrosModelStep(zeros(3, 1), zeros(3, 1), eye(3), eye(3), 0, 0, zeros(3, 1), dTs);
        testCase.verifyEqual(objStream.State, ui32StateBefore);
    end
end
end

%% Local functions
function dDCM = AxisAngleToDCM_(dAxis, dAngle_rad)
% Rotation matrix of a rotation by dAngle_rad about dAxis (Rodrigues formula).
dUnitAxis = dAxis / norm(dAxis);
dSkew = [0, -dUnitAxis(3), dUnitAxis(2); dUnitAxis(3), 0, -dUnitAxis(1); -dUnitAxis(2), dUnitAxis(1), 0];
dDCM = eye(3) + sin(dAngle_rad) * dSkew + (1 - cos(dAngle_rad)) * (dSkew * dSkew);
end
