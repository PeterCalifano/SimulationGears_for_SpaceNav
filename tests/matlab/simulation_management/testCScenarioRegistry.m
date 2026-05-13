classdef testCScenarioRegistry < matlab.unittest.TestCase
    methods (Test)
        function testAliasResolution(testCase)
            [enumScenarioName, charCanonicalName] = CScenarioRegistry.ResolveScenario("Dydimos");

            testCase.verifyEqual(enumScenarioName, EnumScenarioName.Didymos);
            testCase.verifyEqual(charCanonicalName, "Didymos");

            [enumScenarioName, charCanonicalName] = CScenarioRegistry.ResolveScenario("Custom");
            testCase.verifyEqual(enumScenarioName, EnumScenarioName.FromShape);
            testCase.verifyEqual(charCanonicalName, "FromShape");

            [enumScenarioName, charCanonicalName] = CScenarioRegistry.ResolveScenario("passthrough");
            testCase.verifyEqual(enumScenarioName, EnumScenarioName.NotDefined);
            testCase.verifyEqual(charCanonicalName, "NotDefined");
        end

        function testKnownConstantsScaleWithUnits(testCase)
            strMoonM = CScenarioRegistry.GetScenarioSpec("Moon", charLengthUnits="m");
            strMoonKm = CScenarioRegistry.GetScenarioSpec("Moon", charLengthUnits="km");

            testCase.verifyEqual(strMoonM.dGravParam, 4.902800122445300e12, RelTol=1e-15);
            testCase.verifyEqual(strMoonKm.dGravParam, 4.902800122445300e3, RelTol=1e-15);
            testCase.verifyEqual(strMoonM.dReferenceRadius, 1.7374e6, RelTol=1e-15);
            testCase.verifyEqual(strMoonKm.dReferenceRadius, 1.7374e3, RelTol=1e-15);
        end

        function testModifiedVariantsAreExplicitScenarios(testCase)
            strApophis = CScenarioRegistry.GetScenarioSpec("Apophis", charLengthUnits="m");
            strApophisElongated = CScenarioRegistry.GetScenarioSpec(EnumScenarioName.ApophisElongated, ...
                charLengthUnits="m");
            strItokawaModified = CScenarioRegistry.GetScenarioSpec("ItokawaModified", charLengthUnits="m");

            testCase.verifyEqual(strApophis.enumScenarioName, EnumScenarioName.Apophis);
            testCase.verifyEqual(strApophis.charCanonicalName, "Apophis");
            testCase.verifyEqual(strApophisElongated.enumScenarioName, EnumScenarioName.ApophisElongated);
            testCase.verifyEqual(strApophisElongated.charCanonicalName, "ApophisElongated");
            testCase.verifyTrue(contains(strApophisElongated.charDefaultShapeRelativePath, "Elongated"));
            testCase.verifyEqual(strItokawaModified.enumScenarioName, EnumScenarioName.ItokawaModified);
            testCase.verifyEqual(strItokawaModified.charCanonicalName, "ItokawaModified");
            testCase.verifyEqual(strItokawaModified.charShapeSourceType, "obj");
        end

        function testRegistrySHDegreeLimits(testCase)
            strEros = CScenarioRegistry.GetScenarioSpec("Eros", charLengthUnits="m");
            testCase.verifyEqual(strEros.strSphericalHarmonics.ui32SourceMaxDegree, uint32(15));
            testCase.verifyEqual(strEros.strSphericalHarmonics.ui32HardcodedMaxDegree, uint32(15));

            [strSHgravityData, strSHmeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData("Moon", uint32(4), "m");
            testCase.verifyTrue(strSHmeta.bHasHardcodedCoefficients);
            testCase.verifyEqual(strSHgravityData.ui32MaxDegree, uint32(4));
            testCase.verifySize(strSHgravityData.dCSlmCoeffCols, [13 2]);

            [strSHgravityData, strSHmeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData("Moon", uint32(17), "m");
            testCase.verifyTrue(strSHmeta.bHasHardcodedCoefficients);
            testCase.verifyEqual(strSHmeta.ui32HardcodedMaxDegree, uint32(16));
            testCase.verifyEmpty(strSHgravityData.dCSlmCoeffCols);
        end

        function testLoadDefaultScenarioDataUsesRegistry(testCase)
            [charTargetName, charTargetFixedFrame, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData( ...
                "Moon", struct(), "bUseKilometersScale", true);

            testCase.verifyEqual(string(charTargetName), "MOON");
            testCase.verifyEqual(string(charTargetFixedFrame), "IAU_MOON");
            testCase.verifyEqual(strDynParams.strMainData.dGM, 4.902800122445300e3, RelTol=1e-15);
            testCase.verifyEqual(strDynParams.strMainData.dRefRadius, 1.7374e3, RelTol=1e-15);
        end

        function testLoadDefaultScenarioDataGetsRegistrySH(testCase)
            [~, ~, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData( ...
                "Moon", struct(), ...
                "bUseKilometersScale", true, ...
                "bAddNonSphericalGravityCoeffs", true, ...
                "ui16MaxSHdegree", uint16(4));

            testCase.verifyEqual(strDynParams.strMainData.ui16MaxSHdegree, uint16(4));
            testCase.verifySize(strDynParams.strMainData.dSHcoeff, [13 2]);
            testCase.verifyTrue(strDynParams.strMainData.strSHmetadata.bHasHardcodedCoefficients);
        end

        function testScenarioDatasetBuilderSmoke(testCase)
            dTimestamps = [10 20 30];
            dStateSC_W = [ones(3, 3); zeros(3, 3)];

            [objDataset, strEnvironmentData, strDynParams, strMetadata] = ...
                CScenarioGenerator.BuildReferenceScenarioDataset("Moon", dStateSC_W, dTimestamps, ...
                bCompleteFromReferences=true, bUseKilometersScale=true);

            testCase.verifyClass(objDataset, "SReferenceImagesDataset");
            testCase.verifyEqual(objDataset.dTimestamps, dTimestamps);
            testCase.verifyEqual(string(strEnvironmentData.charTargetName), "MOON");
            testCase.verifyEqual(strDynParams.strMainData.dGM, strEnvironmentData.dGravParam);
            testCase.verifyTrue(strMetadata.bCompleteFromReferences);
        end
    end
end
