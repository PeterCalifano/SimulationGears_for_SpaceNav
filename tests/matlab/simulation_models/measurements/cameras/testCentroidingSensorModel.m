function objTests = testCentroidingSensorModel()
%% SIGNATURE
% objTests = testCentroidingSensorModel()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Check explicit draws, camera projection and elapsed-time centroid bias dynamics.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 12-10-2026  Pietro Califano, Codex gpt-6.1  Specify owned centroid sensor behavior.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header; readability pass; rename from
%                                               testCentroidSensorModel.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CCentroidingSensorModel.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testProjectionAndExplicitErrors(testCase)
% Project [2;3;10] to [70;96], add white and constant errors, and leave the RNG untouched.
objSensor = CCentroidingSensorModel('dWhiteNoiseSigma', [2; 3], ...
    'dConstantBias', [4; -5]);
strRandomState = rng;

[objNext, dMeasurement, bValid, strDiagnostic] = objSensor.step(10, ...
    [2; 3; 10], [100, 0, 50; 0, 120, 60; 0, 0, 1], [1; -1], [0; 0]);

verifyTrue(testCase, bValid);
verifyEqual(testCase, strDiagnostic.dProjection, [70; 96], 'AbsTol', 1e-12);
verifyEqual(testCase, dMeasurement, [76; 88], 'AbsTol', 1e-12);
verifyEqual(testCase, rng, strRandomState);
verifyFalse(testCase, objSensor.bInitialized);
verifyTrue(testCase, objNext.bInitialized);
end

function testExactElapsedTimeBias(testCase)
% The bias starts at zero, then follows the exact OU transition over the elapsed time.
objSensor = CCentroidingSensorModel('dBiasDrivingSigma', [2; 3], ...
    'dBiasTimeConstant', [4; 8]);
objSensor = objSensor.step(100, [0; 0; 10], eye(3), zeros(2, 1), [9; 9]);
verifyEqual(testCase, objSensor.dCorrelatedBias, zeros(2, 1));

[objNext, ~, ~, strDiagnostic] = objSensor.step(102, [0; 0; 10], eye(3), zeros(2, 1), [1; -1]);
dDecay = exp(-2./[4; 8]);
dVariance = [4; 9].*[4; 8]./2.*(1 - dDecay.^2);
verifyEqual(testCase, objNext.dCorrelatedBias, sqrt(dVariance).*[1; -1], 'AbsTol', 1e-12);
verifyEqual(testCase, strDiagnostic.dBiasVariance, dVariance, 'AbsTol', 1e-12);

objLater = objNext.step(107, [0; 0; 10], eye(3), zeros(2, 1), zeros(2, 1));
verifyEqual(testCase, objLater.dCorrelatedBias, ...
    exp(-5./[4; 8]).*objNext.dCorrelatedBias, 'AbsTol', 1e-12);
end

function testIndependentAxisAndInvalidVisibility(testCase)
% A disabled axis keeps zero bias; the enabled axis evolves while the target is behind the camera.
objSensor = CCentroidingSensorModel('dBiasDrivingSigma', [2; 0], ...
    'dBiasTimeConstant', [4; 0]);
objSensor = objSensor.step(0, [0; 0; 1], eye(3), zeros(2, 1), zeros(2, 1));

[objSensor, dMeasurement, bValid] = objSensor.step(1, [0; 0; -1], eye(3), zeros(2, 1), ones(2, 1));

verifyFalse(testCase, bValid);
verifyTrue(testCase, all(isnan(dMeasurement)));
verifyGreaterThan(testCase, objSensor.dCorrelatedBias(1), 0);
verifyEqual(testCase, objSensor.dCorrelatedBias(2), 0);
end

function testAdmissionAndReset(testCase)
% Reject a driven axis without a time constant and a repeated timestamp; reset returns a new history.
verifyError(testCase, @() CCentroidingSensorModel('dBiasDrivingSigma', [1; 0]), ...
    'CCentroidingSensorModel:InvalidBiasTimeConstant');

objSensor = CCentroidingSensorModel();
objSensor = objSensor.step(1, [0; 0; 1], eye(3), zeros(2, 1), zeros(2, 1));
verifyError(testCase, @() objSensor.step(1, [0; 0; 1], eye(3), zeros(2, 1), zeros(2, 1)), ...
    'CCentroidingSensorModel:NonMonotonicTimestamp');

objReset = objSensor.reset();
verifyFalse(testCase, objReset.bInitialized);
verifyEqual(testCase, objSensor.dLastTimestamp, 1);
end
