classdef testScenarioDataManifests < matlab.unittest.TestCase
    methods (Test)
        function testResolveDataRootDefaultsToSimGearsData(testCase)
            charDataRoot = ResolveSimGearsDataRoot();

            testCase.verifyTrue(endsWith(string(charDataRoot), fullfile("SimulationGears_for_SpaceNav", "data")));
            testCase.verifyTrue(isfolder(charDataRoot));
        end

        function testLoadTrackedScenarioManifest(testCase)
            strManifest = LoadScenarioDataManifest("Eros");

            testCase.verifyEqual(string(strManifest.scenario_name), "Eros");
            testCase.verifyEqual(string(strManifest.default_shape_asset_id), "naif_eros_dsk_q64");
            testCase.verifyTrue(numel(strManifest.assets) >= 2);
        end

        function testManifestValidationFailsOnScenarioMismatch(testCase)
            strManifest = testScenarioDataManifests.buildValidManifest_();
            strManifest.scenario_name = "Moon";

            testCase.verifyError(@() ValidateScenarioDataManifest(strManifest, "Eros"), ...
                "ValidateScenarioDataManifest:ScenarioMismatch");
        end

        function testManifestValidationFailsOnMissingDefaultShape(testCase)
            strManifest = testScenarioDataManifests.buildValidManifest_();
            strManifest.assets.asset_id = "missing";

            testCase.verifyError(@() ValidateScenarioDataManifest(strManifest, "Eros"), ...
                "ValidateScenarioDataManifest:MissingDefaultShape");
        end

        function testManifestValidationFailsOnRegistryDefaultMismatch(testCase)
            strManifest = testScenarioDataManifests.buildValidManifest_();
            strManifest.default_shape_asset_id = "gaskell_eros_shape_v1_1";

            testCase.verifyError(@() ValidateScenarioDataManifest(strManifest, "Eros"), ...
                "ValidateScenarioDataManifest:DefaultShapeMismatch");
        end

        function testManifestValidationFailsWhenRunnableDefaultIsNotRequired(testCase)
            strManifest = testScenarioDataManifests.buildValidManifest_();
            strManifest.assets.required_for_shape_runnable = false;

            testCase.verifyError(@() ValidateScenarioDataManifest(strManifest, "Eros"), ...
                "ValidateScenarioDataManifest:DefaultShapeNotRunnable");
        end

        function testManifestValidationFailsOnRegistryDefaultPathMismatch(testCase)
            strManifest = testScenarioDataManifests.buildValidManifest_();
            strManifest.assets.local_path = "scenarios/Eros/assets/shape/other.bds";

            testCase.verifyError(@() ValidateScenarioDataManifest(strManifest, "Eros"), ...
                "ValidateScenarioDataManifest:DefaultShapePathMismatch");
        end

        function testManifestValidationChecksLocalShapeAssetWhenRequested(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            strManifest = testScenarioDataManifests.buildValidManifest_();

            testCase.verifyError(@() ValidateScenarioDataManifest(strManifest, "Eros", ...
                charDataRootPath=string(fixture.Folder), bRequireLocalAssets=true), ...
                "ValidateScenarioDataManifest:MissingAsset");
        end

        function testFetchScenarioDataDryRunReturnsPlannedAssets(testCase)
            strPlan = FetchScenarioData("Eros", bDryRun=true);

            testCase.verifyEqual(string(strPlan.charScenarioName), "Eros");
            testCase.verifyGreaterThanOrEqual(numel(strPlan.strAssets), 2);
            testCase.verifyTrue(all([strPlan.strAssets.bWouldDownload] | [strPlan.strAssets.bExists]));
        end

        function testAllManifestBackedScenariosHaveAlignedDefaults(testCase)
            cellExpectedManifestBacked = ["Apophis", "ApophisElongated", "Itokawa", "Bennu", ...
                "Didymos", "Eros", "Arrokoth", "Comet67P", "Toutatis"];
            cellManifestBacked = strings(1, 0);

            for charScenarioName = CScenarioRegistry.ListSupportedScenarios()
                strSpec = CScenarioRegistry.GetScenarioSpec(charScenarioName);
                if strlength(string(strSpec.charDataManifestRelativePath)) == 0
                    continue
                end

                cellManifestBacked(end + 1) = string(strSpec.charCanonicalName); %#ok<AGROW>
                strManifest = LoadScenarioDataManifest(strSpec.enumScenarioName);
                testCase.verifyEqual(string(strManifest.default_shape_asset_id), ...
                    string(strSpec.charDefaultShapeAssetId), string(strSpec.charCanonicalName));

                strAssetIds = string({strManifest.assets.asset_id});
                idxDefaultShape = find(strAssetIds == string(strManifest.default_shape_asset_id), 1);
                testCase.verifyNotEmpty(idxDefaultShape, string(strSpec.charCanonicalName));
                testCase.verifyEqual(string(strManifest.assets(idxDefaultShape).local_path), ...
                    string(strSpec.charDefaultShapeRelativePath), string(strSpec.charCanonicalName));

                if any(strcmp(string(strManifest.tags), "shape_runnable"))
                    testCase.verifyTrue(logical(strManifest.assets(idxDefaultShape).required_for_shape_runnable), ...
                        string(strSpec.charCanonicalName));
                end
            end

            testCase.verifyEqual(sort(cellManifestBacked), sort(cellExpectedManifestBacked));
        end

        function testRegisteredSpiceMetaKernelIsDeclaredByManifest(testCase)
            strSpec = CScenarioRegistry.GetScenarioSpec("Apophis");
            strManifest = LoadScenarioDataManifest("Apophis");
            cellAssetPaths = string({strManifest.assets.local_path});

            testCase.verifyTrue(any(cellAssetPaths == ...
                string(strSpec.charDefaultSpiceMetaKernelRelativePath)));
        end

        function testManifestBackedDefaultShapePathsMatchLoaderContracts(testCase)
            for charScenarioName = CScenarioRegistry.ListSupportedScenarios()
                strSpec = CScenarioRegistry.GetScenarioSpec(charScenarioName);
                if strlength(string(strSpec.charDataManifestRelativePath)) == 0
                    continue
                end

                [~, ~, charExtension] = fileparts(string(strSpec.charDefaultShapeRelativePath));
                switch string(strSpec.charShapeSourceType)
                    case "dsk"
                        testCase.verifyTrue(any(strcmpi(string(charExtension), [".bds", ".dsk"])), ...
                            string(strSpec.charCanonicalName));
                    case "obj"
                        testCase.verifyEqual(lower(string(charExtension)), ".obj", ...
                            string(strSpec.charCanonicalName));
                    otherwise
                        testCase.verifyFail("Unsupported loader contract for " + string(strSpec.charCanonicalName));
                end
            end
        end

        function testLegacyMetadataScenariosAreNotManifestBacked(testCase)
            for charScenarioName = ["Moon", "Mars", "Ceres", "Earth", "FromShape", "NotDefined"]
                strSpec = CScenarioRegistry.GetScenarioSpec(charScenarioName);
                testCase.verifyEqual(strlength(string(strSpec.charDataManifestRelativePath)), 0, charScenarioName);
                testCase.verifyEmpty(string(strSpec.cellTags), charScenarioName);
            end

            testCase.verifyError(@() LoadScenarioDataManifest("Moon"), ...
                "LoadScenarioDataManifest:ManifestUnavailable");
        end
    end

    methods (Static, Access = private)
        function strManifest = buildValidManifest_()
            strAsset = struct( ...
                "asset_id", "naif_eros_dsk_q64", ...
                "asset_type", "shape", ...
                "local_path", "scenarios/Eros/assets/shape/near-a-msi-5-erosshape-v1_0_64q.bds", ...
                "source_url", "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/dsk/asteroids/eros/near-a-msi-5-erosshape-v1_0_64q.cmt", ...
                "download_url", "", ...
                "sha256", "", ...
                "size_gb", 0.005, ...
                "fidelity", "loadable_fast_fallback", ...
                "required_for_shape_runnable", true);
            strManifest = struct( ...
                "schema_version", 1, ...
                "scenario_name", "Eros", ...
                "canonical_name", "Eros", ...
                "aliases", {{"433 Eros", "(433) Eros", "Eros"}}, ...
                "confidence", "high", ...
                "tags", {{"shape_runnable"}}, ...
                "default_shape_asset_id", "naif_eros_dsk_q64", ...
                "assets", strAsset);
        end
    end
end
