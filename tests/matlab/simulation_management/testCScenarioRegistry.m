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

        function testApophisElongatedIsFirstClassAndItokawaModifiedIsUnsupported(testCase)
            strApophis = CScenarioRegistry.GetScenarioSpec("Apophis", charLengthUnits="m");
            strApophisElongated = CScenarioRegistry.GetScenarioSpec(EnumScenarioName.ApophisElongated, ...
                charLengthUnits="m");

            testCase.verifyEqual(strApophis.enumScenarioName, EnumScenarioName.Apophis);
            testCase.verifyEqual(strApophis.charCanonicalName, "Apophis");
            testCase.verifyEqual(strApophisElongated.enumScenarioName, EnumScenarioName.ApophisElongated);
            testCase.verifyEqual(strApophisElongated.charCanonicalName, "ApophisElongated");
            testCase.verifyTrue(contains(strApophisElongated.charDefaultShapeRelativePath, "Elongated"));
            testCase.verifyEqual(strApophisElongated.charScenarioTag, "ApophisElongated");
            testCase.verifyNotEqual(strApophisElongated.dEllipsoidAxes_m, strApophis.dEllipsoidAxes_m);
            testCase.verifyEqual(strApophisElongated.dEllipsoidAxes_m, ...
                [241.42196644, 152.28733120, 147.42347991], RelTol=1e-10);
            testCase.verifyEqual(strApophisElongated.dTargetShapeMatrix_OF, ...
                diag(1.0 ./ (strApophisElongated.dEllipsoidAxes_m .^ 2)), RelTol=1e-14);

            testCase.verifyError(@() CScenarioRegistry.ResolveScenario("ItokawaModified"), ...
                "CScenarioRegistry:UnsupportedScenario");
            testCase.verifyError(@() CScenarioRegistry.ResolveScenario("ModifiedItokawa"), ...
                "CScenarioRegistry:UnsupportedScenario");
        end

        function testNewTaggedScenarioAliasesResolve(testCase)
            [enumScenarioName, charCanonicalName] = CScenarioRegistry.ResolveScenario("67P");
            testCase.verifyEqual(enumScenarioName, EnumScenarioName.Comet67P);
            testCase.verifyEqual(charCanonicalName, "Comet67P");

            [enumScenarioName, charCanonicalName] = CScenarioRegistry.ResolveScenario("433 Eros");
            testCase.verifyEqual(enumScenarioName, EnumScenarioName.Eros);
            testCase.verifyEqual(charCanonicalName, "Eros");

            testCase.verifyEqual(CScenarioRegistry.ResolveScenario("Arrokoth"), EnumScenarioName.Arrokoth);
            testCase.verifyEqual(CScenarioRegistry.ResolveScenario("Toutatis"), EnumScenarioName.Toutatis);
        end

        function testLengthUnitsAcceptEnum(testCase)
            strMoonKm = CScenarioRegistry.GetScenarioSpec("Moon", charLengthUnits=EnumLengthUnits.km);

            testCase.verifyEqual(strMoonKm.charLengthUnits, "km");
            testCase.verifyEqual(strMoonKm.dReferenceRadius, 1.7374e3, RelTol=1e-15);
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

        function testItokawaHardcodedDegree16IsOneFamily(testCase)
            [strDegree4Km, strMeta4] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Itokawa", uint32(4), "km");
            [strDegree8Km, strMeta8] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Itokawa", uint32(8), "km");
            [strDegree16Km, strMeta16] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Itokawa", uint32(16), "km");
            [strDegree16M, ~] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Itokawa", uint32(16), "m");

            testCase.verifyTrue(strMeta4.bHasHardcodedCoefficients);
            testCase.verifyTrue(strMeta8.bHasHardcodedCoefficients);
            testCase.verifyTrue(strMeta16.bHasHardcodedCoefficients);
            testCase.verifyEqual(strMeta16.ui32HardcodedMaxDegree, uint32(16));
            testCase.verifySize(strDegree4Km.dCSlmCoeffCols, [13 2]);
            testCase.verifySize(strDegree8Km.dCSlmCoeffCols, [43 2]);
            testCase.verifySize(strDegree16Km.dCSlmCoeffCols, [151 2]);
            testCase.verifyEqual(strDegree16Km.dCSlmCoeffCols(2, :), ...
                [-3.28791060362629170e-01, 0.0], AbsTol=0.0);
            testCase.verifyEqual(strDegree16Km.dCSlmCoeffCols(43, :), ...
                [2.50382866678236400e-07, 5.31810290658916300e-08], ...
                AbsTol=0.0);
            testCase.verifyEqual(strDegree16Km.dCSlmCoeffCols(151, :), ...
                [2.95874237989475730e-17, 2.79051944763490740e-17], ...
                AbsTol=0.0);
            testCase.verifyEqual(strDegree4Km.dCSlmCoeffCols, ...
                strDegree16Km.dCSlmCoeffCols(1:13, :), ...
                AbsTol=0.0);
            testCase.verifyEqual(strDegree8Km.dCSlmCoeffCols, ...
                strDegree16Km.dCSlmCoeffCols(1:43, :), ...
                AbsTol=0.0);
            testCase.verifyEqual(strDegree16M.dCSlmCoeffCols, ...
                strDegree16Km.dCSlmCoeffCols, AbsTol=0.0);
            testCase.verifyEqual(strDegree16M.dGravParam, ...
                1.0e9 * strDegree16Km.dGravParam, RelTol=1.0e-14);
            testCase.verifyEqual(strDegree16M.dBodyRadiusRef, ...
                1.0e3 * strDegree16Km.dBodyRadiusRef, RelTol=1.0e-14);

            [~, ~, strDynParams] = ...
                CScenarioGenerator.LoadDefaultScenarioData( ...
                    "Itokawa", struct(), ...
                    "bUseKilometersScale", true, ...
                    "bAddNonSphericalGravityCoeffs", true, ...
                    "ui16MaxSHdegree", uint16(16));
            testCase.verifyEqual(strDynParams.strMainData.ui16MaxSHdegree, ...
                uint16(16));
            testCase.verifyEqual(strDynParams.strMainData.dSHcoeff, ...
                strDegree16Km.dCSlmCoeffCols, AbsTol=0.0);
        end

        function testItokawaDegreeAboveHardcodedLimitIsRejected(testCase)
            [strUnavailableData, strMetadata] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Itokawa", uint32(17), "km");

            testCase.verifyEqual(strMetadata.ui32HardcodedMaxDegree, ...
                uint32(16));
            testCase.verifyEmpty(strUnavailableData.dCSlmCoeffCols);
            testCase.verifyError(@() ...
                CScenarioGenerator.LoadDefaultScenarioData( ...
                    "Itokawa", struct(), ...
                    "bUseKilometersScale", true, ...
                    "bAddNonSphericalGravityCoeffs", true, ...
                    "ui16MaxSHdegree", uint16(17)), ...
                "CScenarioGenerator:RegistrySHUnavailable");
        end

        function testShapeModelConsumesHardcodedItokawaDegree16(testCase)
            objShapeModel = CShapeModel( ...
                "file_obj", "", "km", "km", true, ...
                "Itokawa", false);

            [objShapeModel, strShapeGravityData] = ...
                objShapeModel.BuildAndSetSphericalHarmonicsGravityData( ...
                    uint32(16), charMode="registry");
            strCachedGravityData = ...
                objShapeModel.getSphericalHarmonicsGravityData();
            [strRegistryGravityData, ~] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Itokawa", uint32(16), "km");

            testCase.verifyEqual(strShapeGravityData.dCSlmCoeffCols, ...
                strRegistryGravityData.dCSlmCoeffCols, AbsTol=0.0);
            testCase.verifyEqual(strCachedGravityData.dCSlmCoeffCols, ...
                strRegistryGravityData.dCSlmCoeffCols, AbsTol=0.0);
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
            testCase.verifyFalse(isfield(strDynParams.strMainData, "strSHmetadata"));
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
