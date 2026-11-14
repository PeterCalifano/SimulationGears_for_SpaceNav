function objTests = testLidarPhysicalValidity
%% SIGNATURE
% objTests = testLidarPhysicalValidity
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify noiseless physical acquisition bounds independently of sensor error, geometry misses and numeric
% output validity. Preserve one existing noise draw per geometric hit, including hits outside the physical
% interval.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-10-2026  Pietro Califano, Codex (GPT-6)  Cover physical 3 km boundaries, units and sensor RNG consumption.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ApplyLidarMeasurementError, LaserRangefinderNoiseModel, LaserRangefinderModel, LaserRangefinderModelArrays.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function setupOnce(~)
charTestDirectory = fileparts(mfilename('fullpath'));
charRepoRoot = fullfile(charTestDirectory, '..', '..', '..', '..', '..');
addpath(fullfile(charRepoRoot, 'matlab'));
SetupSimGears();
end

function testNoiseCannotChangePhysicalRangeSupport(objTestCase)
strOriginalRng = rng;
objCleanup = onCleanup(@() rng(strOriginalRng));
for dLengthScale = [1, 1e-3]
    dInterval = [80.9575; 3000] * dLengthScale;
    for dTrueRange_m = [79.9575, 80.9575, 81.9575, 2999, 3000, 3001]
        for dError_m = [-2, 2]
            % Calibrate a deterministic nonzero white-noise realization to cross the boundary in either
            % direction.
            rng(27102026, 'twister');
            dWhiteDraw = randn;
            dBias = (dError_m - dWhiteDraw) * dLengthScale;
            rng(27102026, 'twister');
            [dMeasurement, bValid, dError] = ApplyLidarMeasurementError( ...
                dTrueRange_m*dLengthScale, true, dLengthScale, dBias, dInterval, true, true);
            verifyEqual(objTestCase, bValid, dTrueRange_m >= 80.9575 && dTrueRange_m <= 3000);
            verifyEqual(objTestCase, dError, dError_m*dLengthScale, 'AbsTol', 1e-12);
            verifyEqual(objTestCase, dMeasurement, (dTrueRange_m+dError_m)*dLengthScale, 'AbsTol', 1e-10);
        end
    end
end
end

function testRawPreparedAndArraySensorsSharePhysicalPolicy(objTestCase)
strOriginalRng = rng;
objCleanup = onCleanup(@() rng(strOriginalRng));
for dLengthScale = [1, 1e-3]
    dVertices = [0, 0, 0; -10, 10, 0; -10, -10, 10] * dLengthScale;
    strRaw = struct('dVerticesPositions', dVertices, 'i32triangVertexPtrs', int32([1; 2; 3]));
    strRayData = BuildTriangleRayData(dVertices, true);
    strPrepared = struct('strRayData', strRayData);
    for dTrueRange_m = [2999, 3000, 3001]
        for dBias_m = [-2, 2]
            for dInterfaceIdx = 1:3
                rng(991, 'twister');
                if dInterfaceIdx < 3
                    strModel = strRaw;
                    if dInterfaceIdx == 2
                        strModel = strPrepared;
                    end
                    [dRange, bHit, bValid, dPoint, dError] = LaserRangefinderModel( ...
                        strModel, [1; 0; 0], [-dTrueRange_m; 0; 0]*dLengthScale, 0, dBias_m*dLengthScale, ...
                        zeros(3, 1), [80.9575; 3000]*dLengthScale, true, false, true);
                else
                    [dRange, bHit, bValid, dPoint, dError] = CallArrayModel_(strRayData, ...
                        dTrueRange_m*dLengthScale, dBias_m*dLengthScale, [80.9575; 3000]*dLengthScale);
                end
                verifyTrue(objTestCase, bHit);
                verifyEqual(objTestCase, bValid, dTrueRange_m <= 3000);
                verifyEqual(objTestCase, dRange, (dTrueRange_m+dBias_m)*dLengthScale, 'AbsTol', 1e-10);
                verifyEqual(objTestCase, dPoint, zeros(3, 1), 'AbsTol', 1e-12);
                verifyEqual(objTestCase, dError, dBias_m*dLengthScale, 'AbsTol', 1e-12);
            end
        end
    end
end
end

function testEveryGeometricHitRetainsOneNoiseDraw(objTestCase)
strOriginalRng = rng;
objCleanup = onCleanup(@() rng(strOriginalRng));
for dTrueRange = [79, 81, 2999, 3000, 3001]
    for dSigma = [0, 1]
        rng(991, 'twister');
        dExpectedError = LaserRangefinderNoiseModel(dSigma, dTrueRange, 2);
        strExpectedRng = rng;
        rng(991, 'twister');
        [~, ~, dError] = ApplyLidarMeasurementError(dTrueRange, true, dSigma, 2, [80.9575; 3000], true, true);
        verifyEqual(objTestCase, dError, dExpectedError);
        verifyEqual(objTestCase, rng, strExpectedRng);
    end
end
rng(991, 'twister');
strBeforeMiss = rng;
[dRange, bValid, dError] = ApplyLidarMeasurementError(NaN, false, 1, 2, [80.9575; 3000], true, true);
verifyEqual(objTestCase, dRange, -1);
verifyFalse(objTestCase, bValid);
verifyEqual(objTestCase, dError, 0);
verifyEqual(objTestCase, rng, strBeforeMiss);
end

function testNumericOutputValidityIsSeparateFromPhysicalBounds(objTestCase)
strOriginalRng = rng;
objCleanup = onCleanup(@() rng(strOriginalRng));
for bChecks = [false, true]
    for dBias = [-2, Inf, NaN]
        [dRange, bValid] = ApplyLidarMeasurementError(1, true, 0, dBias, [0; 3000], true, bChecks);
        verifyFalse(objTestCase, bValid);
        verifyTrue(objTestCase, ~isfinite(dRange) || dRange < 0);
    end
    [~, bInvalidTruth] = ApplyLidarMeasurementError(NaN, true, 0, 0, [0; 3000], false, bChecks);
    verifyFalse(objTestCase, bInvalidTruth);
end
[dRange, bValid] = ApplyLidarMeasurementError(3001, true, 0, -2, [0; 3000], true, false);
verifyTrue(objTestCase, bValid);
verifyEqual(objTestCase, dRange, 2999);
end

function [dRange, bHit, bValid, dPoint, dError] = CallArrayModel_(strData, dTrueRange, dBias, dInterval)
%% SIGNATURE
% [dRange,bHit,bValid,dPoint,dError] = CallArrayModel_(strData,dTrueRange,dBias,dInterval)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Exercise the exact direct-array entry point compiled by CodegenTriangleTracing.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strData     Prepared single-triangle geometry.
% dTrueRange  Sensor-to-plane distance in the active length unit.
% dBias       Deterministic sensor bias in that unit.
% dInterval   Inclusive physical acquisition interval.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dRange      Measured range.
% bHit        Geometric intersection flag.
% bValid      Physical/numeric validity flag.
% dPoint      Truth intersection.
% dError      Realized measurement error.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-10-2026  Pietro Califano, Codex (GPT-6)  Cover the prepared MEX source boundary.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% LaserRangefinderModelArrays.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strData (1,1) struct
    dTrueRange (1,1) double
    dBias (1,1) double
    dInterval (2,1) double
end

arguments (Output)
    dRange (1,1) double
    bHit (1,1) logical
    bValid (1,1) logical
    dPoint (3,1) double
    dError (1,1) double
end

% Trace a +X beam from dTrueRange before the triangle at x = 0: zero white noise, the given bias, checks on.
[dRange, bHit, bValid, dPoint, dError] = LaserRangefinderModelArrays( ...
    strData.ui32TriangleCount, strData.dVertex0, strData.dEdge1, strData.dEdge2, ...
    strData.dNodeMin, strData.dNodeMax, strData.ui32NodeLeft, strData.ui32NodeRight, ...
    strData.ui32LeafStart, strData.ui32LeafCount, strData.ui32TriangleOrder, strData.bUseBvh, ...
    [1; 0; 0], [-dTrueRange; 0; 0], 0, dBias, dInterval, true, true);
end
