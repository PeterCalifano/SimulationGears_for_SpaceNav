classdef testSSphericalHarmonicsGravityData < matlab.unittest.TestCase
    %% DESCRIPTION
    % Regression tests for the serialized spherical-harmonics gravity data schema.

    methods (Test)

        function testMatSerializationRoundTripPreservesSchema(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "moon_sh_data.mat");

            objSHdata.saveToFile(charFilePath);
            objLoaded = SSphericalHarmonicsGravityData.fromFile(charFilePath);
            strSaved = objLoaded.toSavedStruct();
            strGravityData = objLoaded.toGravityDataStruct(uint32(2));

            testCase.verifyClass(objLoaded, "SSphericalHarmonicsGravityData");
            testCase.verifyEqual(string(strSaved.strDocumentationHeader.charSavedDataConvention), ...
                "SimulationGears.SSphericalHarmonicsGravityData.v01");
            testCase.verifyEqual(objLoaded.enumScenarioName, EnumScenarioName.Moon);
            testCase.verifyEqual(objLoaded.enumLengthUnits, EnumLengthUnits.km);
            testCase.verifyEqual(strGravityData.ui32MaxDegree, uint32(2));
            testCase.verifyEqual(strGravityData.dCSlmCoeffCols, objSHdata.dCSlmCoeffCols, ...
                "AbsTol", 0.0);
            testCase.verifyFalse(strGravityData.bRegistryBacked);
        end

        function testNormalizedCoefficientStorageFailsFast(testCase)
            strInvalid = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_().toSavedStruct();
            strInvalid.charNormalization = "normalized";

            testCase.verifyError(@() SSphericalHarmonicsGravityData.fromSavedStruct(strInvalid), ...
                "SSphericalHarmonicsGravityData:InvalidSchema");
        end

        function testJsonSerializationRoundTripPreservesSchema(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "moon_sh_data.json");

            objSHdata.saveToFile(charFilePath);
            objLoaded = SSphericalHarmonicsGravityData.fromFile(charFilePath);

            testCase.verifyClass(objLoaded, "SSphericalHarmonicsGravityData");
            testCase.verifyEqual(objLoaded.dCSlmCoeffCols, objSHdata.dCSlmCoeffCols, ...
                "AbsTol", 0.0);
            testCase.verifyEqual(objLoaded.dGravConst, 6.67430e-20, ...
                "RelTol", 1.0e-14);
        end

        function testConstructorAcceptsEnumScenarioAndLengthUnits(testCase)
            objSHdata = SSphericalHarmonicsGravityData( ...
                charScenarioName=EnumScenarioName.Moon, ...
                charLengthUnits=EnumLengthUnits.km, ...
                ui32MaxDegree=uint32(2), ...
                dCSlmCoeffCols=zeros(4, 2), ...
                dGravParam=42.0, ...
                dBodyRadiusRef=7.0);

            testCase.verifyEqual(objSHdata.enumScenarioName, EnumScenarioName.Moon);
            testCase.verifyEqual(objSHdata.enumLengthUnits, EnumLengthUnits.km);
            testCase.verifyEqual(objSHdata.charScenarioName, 'Moon');
            testCase.verifyEqual(objSHdata.charLengthUnits, 'km');
        end

        function testReferenceRadiusRescalingPreservesExteriorField(testCase)
            strInputModel = ...
                testSSphericalHarmonicsGravityData.buildMoonDegree2Data_().toGravityDataStruct();
            dTargetRadius = 14.0;

            strRescaledModel = RescaleSphericalHarmonicsReferenceRadius( ...
                strInputModel, dTargetRadius);

            testCase.verifyEqual(strRescaledModel.dBodyRadiusRef, ...
                dTargetRadius, AbsTol=0.0);
            testCase.verifyEqual(strRescaledModel.dCSlmCoeffCols(1, :), ...
                0.5 .* strInputModel.dCSlmCoeffCols(1, :), AbsTol=0.0);
            testCase.verifyEqual(strRescaledModel.dCSlmCoeffCols(2:end, :), ...
                0.25 .* strInputModel.dCSlmCoeffCols(2:end, :), AbsTol=0.0);

            dSamplePositions_TB = [30.0, 40.0; 2.0, -3.0; 1.0, 4.0];
            [dPotentialInput, dAccelerationInput] = ...
                EvalExtSphHarmSamplesInTargetFrame( ...
                    dSamplePositions_TB, strInputModel.ui32MaxDegree, ...
                    strInputModel.dCSlmCoeffCols, strInputModel.dGravParam, ...
                    strInputModel.dBodyRadiusRef);
            [dPotentialRescaled, dAccelerationRescaled] = ...
                EvalExtSphHarmSamplesInTargetFrame( ...
                    dSamplePositions_TB, strRescaledModel.ui32MaxDegree, ...
                    strRescaledModel.dCSlmCoeffCols, ...
                    strRescaledModel.dGravParam, ...
                    strRescaledModel.dBodyRadiusRef);

            testCase.verifyEqual(dPotentialRescaled, dPotentialInput, ...
                RelTol=1.0e-14);
            testCase.verifyEqual(dAccelerationRescaled, dAccelerationInput, ...
                RelTol=1.0e-14);
        end

        function testUnknownSchemaFieldFailsFast(testCase)
            strInvalid = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_().toSavedStruct();
            strInvalid.dUnexpected = 1.0;

            testCase.verifyError(@() SSphericalHarmonicsGravityData.fromSavedStruct(strInvalid), ...
                "SSphericalHarmonicsGravityData:InvalidSchema");
        end

        function testRequestedDegreeAboveFileMaxFailsFast(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();

            testCase.verifyError(@() objSHdata.toGravityDataStruct(uint32(3)), ...
                "SSphericalHarmonicsGravityData:RequestedDegreeUnavailable");
        end

        function testCoefficientRowCountMismatchFailsFast(testCase)
            strInvalid = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_().toSavedStruct();
            strInvalid.dCSlmCoeffCols = strInvalid.dCSlmCoeffCols(1:end-1, :);

            testCase.verifyError(@() SSphericalHarmonicsGravityData.fromSavedStruct(strInvalid), ...
                "SSphericalHarmonicsGravityData:InvalidCoefficientRows");
        end

        function testLengthUnitMismatchFailsFast(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            strScenarioSpec = CScenarioRegistry.GetScenarioSpec("Moon", charLengthUnits="m");

            testCase.verifyError(@() objSHdata.validate(uint32(2), ...
                "strScenarioSpec", strScenarioSpec, ...
                "bRequireScenarioMatch", true), ...
                "SSphericalHarmonicsGravityData:LengthUnitMismatch");
        end

        function testInvalidScaledGravConstFailsFast(testCase)
            strInvalid = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_().toSavedStruct();
            strInvalid.dGravConst = 6.67430e-11;

            testCase.verifyError(@() SSphericalHarmonicsGravityData.fromSavedStruct(strInvalid), ...
                "SSphericalHarmonicsGravityData:InvalidSchema");
        end

        function testLoadDefaultScenarioDataUsesFileBackedCoefficients(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "moon_file_backed_sh.mat");
            objSHdata.saveToFile(charFilePath);

            [~, ~, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData( ...
                "Moon", struct(), ...
                "bUseKilometersScale", true, ...
                "bAddNonSphericalGravityCoeffs", true, ...
                "charSpherHarmCoeffInputFileName", charFilePath, ...
                "ui16MaxSHdegree", uint16(2));

            testCase.verifyEqual(strDynParams.strMainData.ui16MaxSHdegree, uint16(2));
            testCase.verifyEqual(strDynParams.strMainData.dSHcoeff, objSHdata.dCSlmCoeffCols, ...
                "AbsTol", 0.0);
            testCase.verifyEqual(strDynParams.strMainData.dGM, objSHdata.dGravParam, ...
                "AbsTol", 0.0);
            testCase.verifyEqual(strDynParams.strMainData.dRefRadius, objSHdata.dBodyRadiusRef, ...
                "AbsTol", 0.0);
            testCase.verifyFalse(isfield(strDynParams.strMainData, "strSHmetadata"));
        end

        function testLoadDefaultScenarioDataFileBackedDefaultsToDegreeFour(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree4Data_();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "moon_file_backed_default_degree4.mat");
            objSHdata.saveToFile(charFilePath);

            [~, ~, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData( ...
                "Moon", struct(), ...
                "bUseKilometersScale", true, ...
                "bAddNonSphericalGravityCoeffs", true, ...
                "charSpherHarmCoeffInputFileName", charFilePath);

            testCase.verifyEqual(strDynParams.strMainData.ui16MaxSHdegree, uint16(4));
            testCase.verifyEqual(strDynParams.strMainData.dSHcoeff, objSHdata.dCSlmCoeffCols, ...
                "AbsTol", 0.0);
        end

        function testFileBackedDefaultDoesNotUseFileMaxDegree(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "moon_file_backed_degree2_not_default.mat");
            objSHdata.saveToFile(charFilePath);

            testCase.verifyError(@() CScenarioGenerator.LoadDefaultScenarioData( ...
                "Moon", struct(), ...
                "bUseKilometersScale", true, ...
                "bAddNonSphericalGravityCoeffs", true, ...
                "charSpherHarmCoeffInputFileName", charFilePath), ...
                "SSphericalHarmonicsGravityData:RequestedDegreeUnavailable");
        end

        function testLoadDefaultScenarioDataZeroDegreeSkipsSHData(testCase)
            [~, ~, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData( ...
                "Moon", struct(), ...
                "bUseKilometersScale", true, ...
                "bAddNonSphericalGravityCoeffs", true, ...
                "ui16MaxSHdegree", uint16(0));

            testCase.verifyEmpty(strDynParams.strMainData.dSHcoeff);
            testCase.verifyEmpty(strDynParams.strMainData.ui16MaxSHdegree);
            testCase.verifyFalse(isfield(strDynParams.strMainData, "strSHmetadata"));
        end

        function testLoadDefaultScenarioDataRejectsScenarioMismatch(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            objSHdata.charScenarioName = "Itokawa";
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "wrong_body_sh.mat");
            objSHdata.saveToFile(charFilePath);

            testCase.verifyError(@() CScenarioGenerator.LoadDefaultScenarioData( ...
                "Moon", struct(), ...
                "bUseKilometersScale", true, ...
                "bAddNonSphericalGravityCoeffs", true, ...
                "charSpherHarmCoeffInputFileName", charFilePath, ...
                "ui16MaxSHdegree", uint16(2)), ...
                "SSphericalHarmonicsGravityData:ScenarioMismatch");
        end

        function testLoadSpherHarmCoefficientsUsesFileBackedCoefficients(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "moon_file_backed_coeffs.mat");
            objSHdata.saveToFile(charFilePath);

            [dCSlmCoeffCols, ui16MaxSHdegree] = CScenarioGenerator.LoadSpherHarmCoefficients( ...
                "Moon", charFilePath);

            testCase.verifyEqual(ui16MaxSHdegree, uint16(2));
            testCase.verifyEqual(dCSlmCoeffCols, objSHdata.dCSlmCoeffCols, ...
                "AbsTol", 0.0);
        end

        function testBuildReferenceScenarioDatasetForwardsFileBackedCoefficients(testCase)
            objSHdata = testSSphericalHarmonicsGravityData.buildMoonDegree2Data_();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charFilePath = fullfile(string(fixture.Folder), "moon_reference_dataset_coeffs.mat");
            objSHdata.saveToFile(charFilePath);

            dStateSC_W = zeros(6, 2);
            dTimestamps = [0.0, 1.0];

            [~, ~, strDynParams] = CScenarioGenerator.BuildReferenceScenarioDataset( ...
                "Moon", dStateSC_W, dTimestamps, ...
                "bCompleteFromReferences", true, ...
                "bUseKilometersScale", true, ...
                "bAddNonSphericalGravityCoeffs", true, ...
                "charSpherHarmCoeffInputFileName", charFilePath, ...
                "ui16MaxSHdegree", uint16(2));

            testCase.verifyEqual(strDynParams.strMainData.ui16MaxSHdegree, uint16(2));
            testCase.verifyEqual(strDynParams.strMainData.dSHcoeff, objSHdata.dCSlmCoeffCols, ...
                "AbsTol", 0.0);
            testCase.verifyEqual(strDynParams.strMainData.dGM, objSHdata.dGravParam, ...
                "AbsTol", 0.0);
        end

    end

    methods (Static, Access = private)

        function objSHdata = buildMoonDegree2Data_()
            dCoeffRows = zeros(4, 2);
            dCoeffRows(2, 1) = -0.125;
            dCoeffRows(4, 2) = 0.03125;

            objSHdata = SSphericalHarmonicsGravityData( ...
                charScenarioName="Moon", ...
                charLengthUnits="km", ...
                ui32MaxDegree=uint32(2), ...
                dCSlmCoeffCols=dCoeffRows, ...
                dGravParam=42.0, ...
                dBodyRadiusRef=7.0, ...
                charSource="unit-test");
        end

        function objSHdata = buildMoonDegree4Data_()
            dCoeffRows = zeros(13, 2);
            dCoeffRows(2, 1) = -0.125;
            dCoeffRows(4, 2) = 0.03125;
            dCoeffRows(9, 1) = 0.0625;

            objSHdata = SSphericalHarmonicsGravityData( ...
                charScenarioName="Moon", ...
                charLengthUnits="km", ...
                ui32MaxDegree=uint32(4), ...
                dCSlmCoeffCols=dCoeffRows, ...
                dGravParam=42.0, ...
                dBodyRadiusRef=7.0, ...
                charSource="unit-test");
        end

    end
end
