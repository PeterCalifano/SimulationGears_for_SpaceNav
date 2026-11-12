classdef testCScenarioRegistry < matlab.unittest.TestCase
    methods (Test)
        function testErosOwnsKernelPathAndPreservesCampaignRoutes(testCase)
            strEros = CScenarioRegistry.GetScenarioSpec("Eros");
            testCase.verifyEqual(string(strEros.charDefaultSpiceMetaKernelRelativePath), ...
                fullfile("assets", "bodies", "eros", "spice", "mk", "metakernel.mk"));
            for charBody = ["Itokawa", "Bennu"]
                strCampaign = CScenarioRegistry.GetScenarioSpec(charBody);
                testCase.verifyEqual(string(strCampaign.charDefaultSpiceMetaKernelRelativePath), "");
            end
            testCase.verifyEqual(strEros.dReferenceRadius_m, 16000);
        end

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

        function testApophisOwnsDefaultSpiceMetaKernel(testCase)
            strApophis = CScenarioRegistry.GetScenarioSpec("Apophis");
            strItokawa = CScenarioRegistry.GetScenarioSpec("Itokawa");

            testCase.verifyEqual(string(strApophis.charDefaultSpiceMetaKernelRelativePath), ...
                fullfile("scenarios", "Apophis", "assets", "spice", "mk", "metakernel.mk"));
            testCase.verifyEqual(string(strItokawa.charDefaultSpiceMetaKernelRelativePath), "");
        end

        function testApophisEmbeddedGravityUsesOneDegree16Family(testCase)
            [strDegree4Km, strMeta4] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Apophis", uint32(4), "km");
            [strDegree8Km, strMeta8] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Apophis", uint32(8), "km");
            [strDegree16Km, strMeta16] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Apophis", uint32(16), "km");
            [strDegree16M, ~] = ...
                CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Apophis", uint32(16), "m");

            testCase.verifyTrue(strMeta4.bHasHardcodedCoefficients);
            testCase.verifyTrue(strMeta8.bHasHardcodedCoefficients);
            testCase.verifyTrue(strMeta16.bHasHardcodedCoefficients);
            testCase.verifyEqual(strMeta16.ui32HardcodedMaxDegree, uint32(16));
            testCase.verifySize(strDegree4Km.dCSlmCoeffCols, [13 2]);
            testCase.verifySize(strDegree8Km.dCSlmCoeffCols, [43 2]);
            testCase.verifySize(strDegree16Km.dCSlmCoeffCols, [151 2]);
            testCase.verifyEqual(strDegree4Km.dCSlmCoeffCols, ...
                strDegree16Km.dCSlmCoeffCols(1:13, :), AbsTol=0.0);
            testCase.verifyEqual(strDegree8Km.dCSlmCoeffCols, ...
                strDegree16Km.dCSlmCoeffCols(1:43, :), AbsTol=0.0);
            testCase.verifyTrue(all(isfinite(strDegree16Km.dCSlmCoeffCols), 'all'));
            testCase.verifyGreaterThan(norm(strDegree16Km.dCSlmCoeffCols), 0.0);
            testCase.verifyEqual(strDegree16M.dCSlmCoeffCols, ...
                strDegree16Km.dCSlmCoeffCols, AbsTol=0.0);
            testCase.verifyEqual(strDegree16M.dGravParam, ...
                1.0e9 * strDegree16Km.dGravParam, RelTol=1.0e-14);
            testCase.verifyEqual(strDegree16M.dBodyRadiusRef, ...
                1.0e3 * strDegree16Km.dBodyRadiusRef, RelTol=1.0e-14);
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

        function testBennuPdsShapeGravityIsCompleteAndScaled(testCase)
            [strDegree4, strMetadata] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Bennu", uint32(4), "km");
            [strDegree16, ~] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Bennu", uint32(16), "km");
            [strDegree16M, ~] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Bennu", uint32(16), "m");

            testCase.verifyTrue(strMetadata.bHasHardcodedCoefficients);
            testCase.verifyEqual(strMetadata.ui32HardcodedMaxDegree, uint32(16));
            testCase.verifySize(strDegree16.dCSlmCoeffCols, [151 2]);
            testCase.verifyEqual(strDegree4.dCSlmCoeffCols, ...
                strDegree16.dCSlmCoeffCols(1:13, :), AbsTol=0);
            testCase.verifyTrue(all(isfinite(strDegree16.dCSlmCoeffCols), 'all'));
            testCase.verifyGreaterThan(norm(strDegree16.dCSlmCoeffCols(2:end, :), 'fro'), 0);
            testCase.verifyLessThan(norm(strDegree16.dCSlmCoeffCols(2:end, :), 'fro'), 1);
            testCase.verifyGreaterThan(strDegree16.dGravParam, 1e-10);
            testCase.verifyLessThan(strDegree16.dGravParam, 1e-7);
            testCase.verifyGreaterThan(strDegree16.dBodyRadiusRef, 0.1);
            testCase.verifyLessThan(strDegree16.dBodyRadiusRef, 1);
            testCase.verifyEqual(strDegree16M.dCSlmCoeffCols, ...
                strDegree16.dCSlmCoeffCols, AbsTol=0);
            testCase.verifyEqual(strDegree16M.dGravParam, ...
                1e9 * strDegree16.dGravParam, RelTol=1e-14);
            testCase.verifyEqual(strDegree16M.dBodyRadiusRef, ...
                1e3 * strDegree16.dBodyRadiusRef, RelTol=1e-14);
        end

        function testPlanetaryMoonEnumValuesAreAppended(testCase)
            % Execution plans store these numeric values, so existing ones must never move.
            cellNames = ["Eros", "Mars", "Toutatis", "Phobos", "Deimos", "Enceladus", "Saturn"];
            ui32Expected = uint32([4, 9, 16, 17, 18, 19, 20]);
            for ui32NameIdx = uint32(1):uint32(numel(cellNames))
                testCase.verifyEqual(uint32(CScenarioRegistry.ResolveScenario(cellNames(ui32NameIdx))), ...
                    ui32Expected(ui32NameIdx), cellNames(ui32NameIdx));
            end
            testCase.verifyEqual(CScenarioRegistry.ResolveScenario("Mars I"), EnumScenarioName.Phobos);
            testCase.verifyEqual(CScenarioRegistry.ResolveScenario("Saturn II"), EnumScenarioName.Enceladus);
        end

        function testPlanetaryMoonSpecsUseDe440Constants(testCase)
            cellNames = ["Phobos", "Deimos", "Enceladus", "Saturn"];
            dExpectedGM_km3s2 = [7.087546066894452e-04, 9.615569648120313e-05, ...
                7.210366688598896, 3.793120623436167e+07];
            for ui32NameIdx = uint32(1):uint32(numel(cellNames))
                strSpec = CScenarioRegistry.GetScenarioSpec(cellNames(ui32NameIdx), charLengthUnits="km");
                testCase.verifyEqual(strSpec.dGravParam, dExpectedGM_km3s2(ui32NameIdx), RelTol=1e-14);
                testCase.verifyEqual(string(strSpec.charTargetFixedFrame), ...
                    "IAU_" + upper(cellNames(ui32NameIdx)));
                testCase.verifyEqual(string(strSpec.charDefaultSpiceMetaKernelRelativePath), ...
                    string(fullfile("assets", "bodies", lower(cellNames(ui32NameIdx)), "spice", "mk", "metakernel.mk")));
            end
        end

        function testMoonShapeFitGravityRows(testCase)
            % Constant-density degree-16 fits of the SBMT plate models.
            [strPhobos, strPhobosMeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Phobos", uint32(16), "km");
            testCase.verifySize(strPhobos.dCSlmCoeffCols, [151 2]);
            testCase.verifyEqual(strPhobos.dBodyRadiusRef, 11.077, RelTol=1e-14);
            testCase.verifyEqual(strPhobos.dCSlmCoeffCols(2, 1), -1.05108858112504158e-01, RelTol=1e-14);
            testCase.verifyEqual(strPhobos.dCSlmCoeffCols(4, 1), 1.52974628173744211e-02, RelTol=1e-14);
            testCase.verifyEqual(strPhobosMeta.charFieldKind, "shape-derived");

            [strDeimos, strDeimosMeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Deimos", uint32(16), "km");
            testCase.verifySize(strDeimos.dCSlmCoeffCols, [151 2]);
            testCase.verifyEqual(strDeimos.dBodyRadiusRef, 6.272, RelTol=1e-14);
            testCase.verifyEqual(strDeimos.dCSlmCoeffCols(2, 1), -1.07207633656657766e-01, RelTol=1e-14);
            testCase.verifyEqual(strDeimos.dCSlmCoeffCols(4, 1), 3.56379914595495520e-02, RelTol=1e-14);
            testCase.verifyEqual(strDeimosMeta.charFieldKind, "shape-derived");
        end

        function testPublishedGravityRowsKeepTheirSources(testCase)
            % Saturn: Iess et al. 2019, C_n0 = -J_n, R 60330 km.
            [strSaturn, strSaturnMeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Saturn", uint32(12), "km");
            testCase.verifySize(strSaturn.dCSlmCoeffCols, [89 2]);
            testCase.verifyEqual(strSaturn.dBodyRadiusRef, 60330, RelTol=1e-14);
            testCase.verifyEqual(strSaturn.dCSlmCoeffCols(2, 1), -16290.573e-6, RelTol=1e-14);
            testCase.verifyEqual(strSaturn.dCSlmCoeffCols(9, 1), 935.314e-6, RelTol=1e-14);
            testCase.verifyEqual(strSaturnMeta.charFieldKind, "measured");

            % Enceladus: Park et al. 2024, Table 8 Case 2, R 256.6 km.
            [strEnceladus, ~] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Enceladus", uint32(3), "km");
            testCase.verifySize(strEnceladus.dCSlmCoeffCols, [8 2]);
            testCase.verifyEqual(strEnceladus.dBodyRadiusRef, 256.6, RelTol=1e-14);
            testCase.verifyEqual(strEnceladus.dCSlmCoeffCols(2, 1), -5477.45e-6, RelTol=1e-14);
            testCase.verifyEqual(strEnceladus.dCSlmCoeffCols(4, :), [1517.90e-6, -275.31e-6], RelTol=1e-14);
            testCase.verifyEqual(strEnceladus.dCSlmCoeffCols(5, 1), 177.82e-6, RelTol=1e-14);

            % Mars: JGMRO_120G, the field of the Konopliv 2026 frame the Mars metakernel loads.
            [strMars, strMarsMeta] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                "Mars", uint32(16), "km");
            testCase.verifySize(strMars.dCSlmCoeffCols, [151 2]);
            testCase.verifyEqual(strMars.dGravParam, 42828.37509235890, RelTol=1e-14);
            testCase.verifyEqual(strMars.dCSlmCoeffCols(2, 1), -1.9566069453409813e-3, RelTol=1e-12);
            testCase.verifyTrue(contains(strMarsMeta.charSource, "JGMRO_120G"));
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

        function testRegistryHarmonicsKeepTheirDimensionalConstants(testCase)
            % A coefficient family is only physically meaningful with its
            % associated GM and reference radius, in either unit system.
            for charLengthUnits = ["km", "m"]
                bUseKilometersScale = charLengthUnits == "km";
                [strGravityData, ~] = CScenarioRegistry.GetSphericalHarmonicsGravityData( ...
                    "Bennu", uint32(4), charLengthUnits);
                [~, ~, strDynParams] = CScenarioGenerator.LoadDefaultScenarioData( ...
                    "Bennu", struct(), ...
                    "bUseKilometersScale", bUseKilometersScale, ...
                    "bAddNonSphericalGravityCoeffs", true, ...
                    "ui16MaxSHdegree", uint16(4));

                testCase.verifyEqual(strDynParams.strMainData.dGM, ...
                    strGravityData.dGravParam, RelTol=1e-14);
                testCase.verifyEqual(strDynParams.strMainData.dRefRadius, ...
                    strGravityData.dBodyRadiusRef, RelTol=1e-14);
                testCase.verifyEqual(strDynParams.strMainData.dSHcoeff, ...
                    strGravityData.dCSlmCoeffCols, AbsTol=0.0);
            end
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
