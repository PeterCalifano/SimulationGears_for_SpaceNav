function objTests = testCameraSensorMetadata()
%% SIGNATURE
% objTests = testCameraSensorMetadata()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate passive detector/radiometric values, spectral layout and camera copies.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objTests  MATLAB function-test suite.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Add the standard header; readability pass.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% SCameraSensorMetadata, CCameraIntrinsics.
% -------------------------------------------------------------------------------------------------------------
objTests = functiontests(localfunctions);
end

function testUnknownValuesRemainEmpty(testCase)
% A default record holds no measurements: every numeric field is empty.
objMetadata = SCameraSensorMetadata();

verifyEmpty(testCase, objMetadata.dGainElectronsPerDN);
verifyEmpty(testCase, objMetadata.dReadNoiseSigmaElectrons);
verifyEmpty(testCase, objMetadata.dFullWellElectrons);
verifyEmpty(testCase, objMetadata.dDetectorBitDepth);
verifyEmpty(testCase, objMetadata.dDarkCurrentElectronsPerSec);
verifyEmpty(testCase, objMetadata.dOffsetElectrons);
verifyEmpty(testCase, objMetadata.dPixelPitchMicrometers);
verifyEmpty(testCase, objMetadata.dFNumber);
verifyEmpty(testCase, objMetadata.dOpticsTransmittance);
verifyEmpty(testCase, objMetadata.dOpticsWavelengthNm);
verifyEmpty(testCase, objMetadata.dOpticsTransmittanceSamples);
verifyEmpty(testCase, objMetadata.dQuantumEfficiencyWavelengthNm);
verifyEmpty(testCase, objMetadata.dQuantumEfficiency);
verifyEmpty(testCase, objMetadata.charWavebandNames);
verifySize(testCase, objMetadata.dWavebandQuantumEfficiency, [0, 0]);
verifyEqual(testCase, objMetadata.charSourceReference, "");
end

function testInvalidValuesAreRejected(testCase)
% Detector scalars reject zero gain, negative noise, infinite full well, fractional bits and vectors.
verifyError(testCase, @() SCameraSensorMetadata('dGainElectronsPerDN', 0), ...
    'MATLAB:validators:mustBePositive');
verifyError(testCase, @() SCameraSensorMetadata('dReadNoiseSigmaElectrons', -1), ...
    'MATLAB:validators:mustBeNonnegative');
verifyError(testCase, @() SCameraSensorMetadata('dFullWellElectrons', Inf), ...
    'MATLAB:validators:mustBeFinite');
verifyError(testCase, @() SCameraSensorMetadata('dDetectorBitDepth', 8.5), ...
    'MATLAB:validators:mustBeInteger');
verifyError(testCase, @() SCameraSensorMetadata('dGainElectronsPerDN', [1, 2]), ...
    'MATLAB:validators:mustBeScalarOrEmpty');
end

function testPartialRadiometricScalarsDoNotInferOrDraw(testCase)
% Supplied scalars are stored as given; missing ones stay empty and the RNG is untouched.
strRngBefore = rng;
objMetadata = SCameraSensorMetadata('dDarkCurrentElectronsPerSec', 0, ...
                                    'dOffsetElectrons', -8, ...
                                    'dPixelPitchMicrometers', [2.2, 3.1], ...
                                    'dFNumber', 2.8);

verifyEqual(testCase, objMetadata.dDarkCurrentElectronsPerSec, 0);
verifyEqual(testCase, objMetadata.dOffsetElectrons, -8);
verifyEqual(testCase, objMetadata.dPixelPitchMicrometers, [2.2, 3.1]);
verifyEqual(testCase, objMetadata.dFNumber, 2.8);
verifyEmpty(testCase, objMetadata.dReadNoiseSigmaElectrons);
verifyEmpty(testCase, objMetadata.dGainElectronsPerDN);
verifyEqual(testCase, rng, strRngBefore);
end

function testInvalidRadiometricScalarsAreRejected(testCase)
% Dark current, offset, pixel pitch, f-number and transmission reject out-of-domain values.
verifyError(testCase, @() SCameraSensorMetadata('dDarkCurrentElectronsPerSec', -1), ...
    'MATLAB:validators:mustBeNonnegative');
verifyError(testCase, @() SCameraSensorMetadata('dOffsetElectrons', Inf), ...
    'MATLAB:validators:mustBeFinite');
verifyError(testCase, @() SCameraSensorMetadata('dOffsetElectrons', 1i), ...
    'MATLAB:validators:mustBeReal');
verifyError(testCase, @() SCameraSensorMetadata('dPixelPitchMicrometers', [1, 0]), ...
    'MATLAB:validators:mustBePositive');
verifyError(testCase, @() SCameraSensorMetadata('dPixelPitchMicrometers', 2), ...
    'SCameraSensorMetadata:InvalidPixelPitch');
verifyError(testCase, @() SCameraSensorMetadata('dFNumber', 0), ...
    'MATLAB:validators:mustBePositive');
verifyError(testCase, @() SCameraSensorMetadata('dOpticsTransmittance', 1.1), ...
    'MATLAB:validators:mustBeInRange');
end

function testConstantResponsesNeedNoWavelengthGrid(testCase)
% Constant QE, band QE and transmission are accepted without a wavelength grid.
objMetadata = SCameraSensorMetadata('dQuantumEfficiency', 0.6, ...
                                    'dWavebandQuantumEfficiency', [0.8, 0.3, 0], ...
                                    'charWavebandNames', ["R", "G", "B"], ...
                                    'dOpticsTransmittance', 0.9);

verifyEmpty(testCase, objMetadata.dQuantumEfficiencyWavelengthNm);
verifyEqual(testCase, objMetadata.dQuantumEfficiency, 0.6);
verifyEqual(testCase, objMetadata.dWavebandQuantumEfficiency, [0.8, 0.3, 0]);
verifyEqual(testCase, objMetadata.charWavebandNames, ["R", "G", "B"]);
verifyEqual(testCase, objMetadata.dOpticsTransmittance, 0.9);
end

function testSampledResponsesRetainSeparateGridsAndBandColumns(testCase)
% Build a sampled QE with two bands, a sampled optical response and its neutral-scalar variant
dBandQe = [0.2, 0.1; 0.4, 0.3; 0.1, 0.8];
objMetadata = SCameraSensorMetadata('dQuantumEfficiencyWavelengthNm', [420, 500, 680], ...
                                    'dQuantumEfficiency', [0.2, 0.4, 0.1], ...
                                    'dWavebandQuantumEfficiency', dBandQe, ...
                                    'charWavebandNames', ["BandA", "BandB"], ...
                                    'dOpticsWavelengthNm', [400, 700], ...
                                    'dOpticsTransmittanceSamples', [0.6, 0.85]);
objExplicitNeutral = SCameraSensorMetadata('dOpticsTransmittance', 1, ...
                                           'dOpticsWavelengthNm', [400, 700], ...
                                           'dOpticsTransmittanceSamples', [0.6, 0.85]);
objUnnamedBands = SCameraSensorMetadata('dWavebandQuantumEfficiency', [0.2, 0.3]);

verifyEqual(testCase, objMetadata.dQuantumEfficiencyWavelengthNm, [420, 500, 680]);
verifyEqual(testCase, objMetadata.dQuantumEfficiency, [0.2, 0.4, 0.1]);
verifyEqual(testCase, objMetadata.dWavebandQuantumEfficiency, dBandQe);
verifyEqual(testCase, objMetadata.dOpticsWavelengthNm, [400, 700]);
verifyEqual(testCase, objMetadata.dOpticsTransmittanceSamples, [0.6, 0.85]);
verifyEmpty(testCase, objMetadata.dOpticsTransmittance);
verifyEqual(testCase, objExplicitNeutral.dOpticsTransmittanceSamples, [0.6, 0.85]);
verifyEmpty(testCase, objUnnamedBands.charWavebandNames);
end

function testInvalidOpticalResponsesAreRejected(testCase)
% Sampled transmission needs a strictly increasing grid of matching length and no non-neutral scalar.
verifyError(testCase, @() SCameraSensorMetadata('dOpticsWavelengthNm', [400, 400], ...
                                                'dOpticsTransmittanceSamples', [0.8, 0.9]), ...
    'SCameraSensorMetadata:InvalidWavelengthGrid');
verifyError(testCase, @() SCameraSensorMetadata('dOpticsWavelengthNm', 400, ...
                                                'dOpticsTransmittanceSamples', 0.8), ...
    'SCameraSensorMetadata:InvalidOpticalResponse');
verifyError(testCase, @() SCameraSensorMetadata('dOpticsWavelengthNm', [400, 700]), ...
    'SCameraSensorMetadata:InvalidOpticalResponse');
verifyError(testCase, @() SCameraSensorMetadata('dOpticsTransmittanceSamples', [0.8, 0.9]), ...
    'SCameraSensorMetadata:InvalidOpticalResponse');
verifyError(testCase, @() SCameraSensorMetadata('dOpticsWavelengthNm', [400, 700], ...
                                                'dOpticsTransmittanceSamples', [0.8, 1.1]), ...
    'MATLAB:validators:mustBeInRange');
verifyError(testCase, @() SCameraSensorMetadata('dOpticsTransmittance', 0.9, ...
                                                'dOpticsWavelengthNm', [400, 700], ...
                                                'dOpticsTransmittanceSamples', [0.8, 0.9]), ...
    'SCameraSensorMetadata:DuplicateOpticalResponse');
end

function testInvalidQuantumEfficiencyIsRejected(testCase)
% QE curves must match the shared grid; empty band matrices must be 0-by-0.
verifyError(testCase, @() SCameraSensorMetadata('dQuantumEfficiency', [0.2, 0.4]), ...
    'SCameraSensorMetadata:InvalidQuantumEfficiency');
verifyError(testCase, @() SCameraSensorMetadata('dQuantumEfficiencyWavelengthNm', [700, 400], ...
                                                'dQuantumEfficiency', [0.2, 0.4]), ...
    'SCameraSensorMetadata:InvalidWavelengthGrid');
verifyError(testCase, @() SCameraSensorMetadata('dQuantumEfficiencyWavelengthNm', [400, 500, 700], ...
                                                'dWavebandQuantumEfficiency', [0.2, 0.4; 0.3, 0.5]), ...
    'SCameraSensorMetadata:InvalidQuantumEfficiency');
verifyError(testCase, @() SCameraSensorMetadata('dWavebandQuantumEfficiency', zeros(0, 2)), ...
    'SCameraSensorMetadata:InvalidQuantumEfficiency');
verifyError(testCase, @() SCameraSensorMetadata('dWavebandQuantumEfficiency', zeros(2, 0)), ...
    'SCameraSensorMetadata:InvalidQuantumEfficiency');
verifyError(testCase, @() SCameraSensorMetadata('dQuantumEfficiency', NaN), ...
    'MATLAB:validators:mustBeFinite');
verifyError(testCase, @() SCameraSensorMetadata('dWavebandQuantumEfficiency', [-0.2, 0.4]), ...
    'MATLAB:validators:mustBeInRange');
end

function testWavebandNamesMustMatchDistinctNonemptyColumns(testCase)
% Band names must be nonempty, unique and one per QE column.
verifyError(testCase, @() SCameraSensorMetadata('charWavebandNames', "R"), ...
    'SCameraSensorMetadata:InvalidWavebandNames');
verifyError(testCase, @() SCameraSensorMetadata('charWavebandNames', ["R", "R"], ...
                                                'dWavebandQuantumEfficiency', [0.3, 0.5]), ...
    'SCameraSensorMetadata:InvalidWavebandNames');
verifyError(testCase, @() SCameraSensorMetadata('charWavebandNames', ["R", ""], ...
                                                'dWavebandQuantumEfficiency', [0.3, 0.5]), ...
    'SCameraSensorMetadata:InvalidWavebandNames');
verifyError(testCase, @() SCameraSensorMetadata('charWavebandNames', string(missing), ...
                                                'dWavebandQuantumEfficiency', 0.3), ...
    'SCameraSensorMetadata:InvalidWavebandNames');
end

function testMetadataDoesNotChangeGeometryAndSurvivesCopy(testCase)
% Attach metadata to a camera and copy it through the toolbox conversion
objCamera = CCameraIntrinsics([500, 510], [50, 40], [80, 100], uint32(1));
dOriginalK = objCamera.K;
objCamera.objSensorMetadata = SCameraSensorMetadata('dGainElectronsPerDN', 100, ...
                                                    'dReadNoiseSigmaElectrons', 0, ...
                                                    'dDarkCurrentElectronsPerSec', 0.5, ...
                                                    'dPixelPitchMicrometers', [2.2, 3.1], ...
                                                    'dQuantumEfficiencyWavelengthNm', [400, 700], ...
                                                    'dQuantumEfficiency', [0.2, 0.5], ...
                                                    'charSourceReference', "synthetic fixture");
objCopy = CCameraIntrinsics.fromCameraIntrinsics(objCamera);

verifyEqual(testCase, objCamera.K, dOriginalK);
verifyEqual(testCase, objCopy.K, dOriginalK);
verifyEqual(testCase, objCopy.objSensorMetadata, objCamera.objSensorMetadata);
verifyEqual(testCase, objCopy.objSensorMetadata.dReadNoiseSigmaElectrons, 0);

% Replacing one value record must not modify another camera's response data.
objCopy.objSensorMetadata = SCameraSensorMetadata();

verifyEqual(testCase, objCamera.objSensorMetadata.dQuantumEfficiency, [0.2, 0.5]);
verifyEmpty(testCase, objCopy.objSensorMetadata.dQuantumEfficiency);
end

function testToolboxCameraCopyRemainsUnknown(testCase)
% Copying a plain toolbox camera keeps K and leaves the sensor metadata unknown.
objCamera = cameraIntrinsics([500, 510], [50, 40], [80, 100]);
objCopy = CCameraIntrinsics.fromCameraIntrinsics(objCamera);

verifyEmpty(testCase, objCopy.objSensorMetadata.dGainElectronsPerDN);
verifyEqual(testCase, objCopy.K, objCamera.K);
end
