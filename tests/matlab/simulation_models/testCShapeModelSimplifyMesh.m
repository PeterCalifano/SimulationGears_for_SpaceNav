classdef testCShapeModelSimplifyMesh < matlab.unittest.TestCase
    methods (Test)
        function testConstructorAppliesLoadTimeKeepFraction(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModelFull = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true);
            objShapeModelReduced = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true, ...
                dMeshSimplifyFactor=0.65);

            testCase.verifyEqual(objShapeModelReduced.dMeshSimplifyFactor, 0.65, "AbsTol", 1e-12);
            testCase.verifyLessThan(size(objShapeModelReduced.ui32triangVertexPtr, 2), ...
                size(objShapeModelFull.ui32triangVertexPtr, 2));
            testCase.verifyLessThan(size(objShapeModelReduced.dVerticesPos, 2), ...
                size(objShapeModelFull.dVerticesPos, 2));
        end

        function testConstructorClampsKeepFractionToUnitInterval(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModelHigh = CShapeModel("file_obj", "", "m", "m", true, "placeholder", false, ...
                dMeshSimplifyFactor=1.7);
            objShapeModelLow = CShapeModel("file_obj", charObjPath, "m", "m", true, "placeholder", true, ...
                dMeshSimplifyFactor=-0.2);

            testCase.verifyEqual(objShapeModelHigh.dMeshSimplifyFactor, 1.0, "AbsTol", 1e-12);
            testCase.verifyEqual(objShapeModelLow.dMeshSimplifyFactor, 0.0, "AbsTol", 1e-12);
            testCase.verifyEqual(size(objShapeModelLow.ui32triangVertexPtr, 2), 0);
            testCase.verifyEqual(size(objShapeModelLow.dVerticesPos, 2), 0);
        end

        function testSimplifyMeshReducesGeometryLoadedFromObj(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModel = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true);

            ui32NumFacesBefore = uint32(size(objShapeModel.ui32triangVertexPtr, 2));
            ui32NumVertsBefore = uint32(size(objShapeModel.dVerticesPos, 2));

            [objShapeModel, strReductionStats] = objShapeModel.SimplifyMesh(35.0);

            ui32NumFacesAfter = uint32(size(objShapeModel.ui32triangVertexPtr, 2));
            ui32NumVertsAfter = uint32(size(objShapeModel.dVerticesPos, 2));

            testCase.verifyLessThan(ui32NumFacesAfter, ui32NumFacesBefore);
            testCase.verifyLessThan(ui32NumVertsAfter, ui32NumVertsBefore);
            testCase.verifyEqual(objShapeModel.ui32NumOfVertices, ui32NumVertsAfter);
            testCase.verifyClass(objShapeModel.ui32triangVertexPtr, "uint32");
            testCase.verifyGreaterThanOrEqual(min(objShapeModel.ui32triangVertexPtr, [], "all"), uint32(1));
            testCase.verifyLessThanOrEqual(max(objShapeModel.ui32triangVertexPtr, [], "all"), ui32NumVertsAfter);
            testCase.verifyGreaterThan(strReductionStats.dAchievedFaceReductionPercent, 20.0);
            testCase.verifyGreaterThan(strReductionStats.dAchievedVertexReductionPercent, 5.0);
        end

        function testObjLoaderParsesNormalOnlyFaceSyntaxWhenVertexFaceOnly(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "normal_only_faces.obj");

            fileId = fopen(charObjPath, "w");
            objCleanup = onCleanup(@() fclose(fileId));
            fprintf(fileId, "v 0 0 0\n");
            fprintf(fileId, "v 1 0 0\n");
            fprintf(fileId, "v 0 1 0\n");
            fprintf(fileId, "vn 0 0 1\n");
            fprintf(fileId, "f 1//1 2//1 3//1\n");
            clear objCleanup

            objShapeModel = CShapeModel("file_obj", charObjPath, "m", "m", true, "normal_only_faces", true);

            testCase.verifyEqual(objShapeModel.ui32triangVertexPtr, uint32([1; 2; 3]));
            testCase.verifyEqual(objShapeModel.ui32NumOfVertices, uint32(3));
        end

        function testSimplifyMeshInvalidatesPolyhedronGravityCache(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "icosphere_mesh.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModel = CShapeModel("file_obj", charObjPath, "m", "m", true, "icosphere_mesh", true);
            objShapeModel = objShapeModel.BuildPolyhedronGravityData();

            [ui32EdgeIds, dEdgeDyadics, dFaceDyadics] = objShapeModel.getPolyhedronGravityData();
            testCase.verifyNotEmpty(ui32EdgeIds);
            testCase.verifyNotEmpty(dEdgeDyadics);
            testCase.verifyNotEmpty(dFaceDyadics);

            objShapeModel = objShapeModel.SimplifyMesh(25.0);

            testCase.verifyError(@() objShapeModel.getPolyhedronGravityData(), "CShapeModel:NoGravityData");
        end

        function testDefineShapeModelPassesLoadTimeKeepFraction(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "moon_shape.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(2));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModelFull = DefineShapeModel("Moon", string(fixture.Folder), string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                dMeshSimplifyFactor=1.0, ...
                bInitSphericalHarmonicsGravityData=false);

            objShapeModelReduced = DefineShapeModel("Moon", string(fixture.Folder), string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                dMeshSimplifyFactor=0.55, ...
                bInitSphericalHarmonicsGravityData=false);

            testCase.verifyEqual(objShapeModelReduced.dMeshSimplifyFactor, 0.55, "AbsTol", 1e-12);
            testCase.verifyLessThan(size(objShapeModelReduced.ui32triangVertexPtr, 2), ...
                size(objShapeModelFull.ui32triangVertexPtr, 2));
            testCase.verifyLessThan(size(objShapeModelReduced.dVerticesPos, 2), ...
                size(objShapeModelFull.dVerticesPos, 2));
        end

        function testDefineShapeModelInitsSHGravityByDefault(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "moon_shape.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModel = DefineShapeModel("Moon", string(fixture.Folder), string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                ui32SphericalHarmonicsGravityMaxFitIterations=uint32(1));

            strSHgravityData = objShapeModel.getSphericalHarmonicsGravityData();

            testCase.verifyEqual(strSHgravityData.ui32MaxDegree, uint32(4));
            testCase.verifyTrue(isfinite(strSHgravityData.dGravParam));
            testCase.verifyGreaterThan(strSHgravityData.dGravParam, 0.0);
            testCase.verifyTrue(strSHgravityData.bRegistryBacked);
        end

        function testDefineShapeModelFromShapeLoadsCustomObj(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "custom_shape.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            [objShapeModel, ~, strMetadata] = DefineShapeModel("FromShape", "", string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                bInitSphericalHarmonicsGravityData=false);

            testCase.verifyTrue(objShapeModel.hasData());
            testCase.verifyEqual(max(objShapeModel.dVerticesPos, [], "all"), max(dVerts, [], "all"), "RelTol", 1e-14);
            testCase.verifyEqual(string(strMetadata.charCanonicalTargetName), "FromShape");
            testCase.verifyEqual(string(strMetadata.strScenarioSpec.charShapeSourceType), "custom_obj");
            testCase.verifyFalse(strMetadata.strPhysicalMetadata.bHasPhysicalMetadata);
        end

        function testDefineShapeModelMoonDefaultShapeUsesSimGearsDataRoot(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charDataRootPath = fullfile(string(fixture.Folder), "simgears_data");
            charBpyRootPath = fullfile(string(fixture.Folder), "render_root");
            charObjPath = fullfile(charDataRootPath, "scenarios", "S6_Moon", "Moon.obj");
            mkdir(fileparts(charObjPath));
            mkdir(charBpyRootPath);

            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            [objShapeModel, ~, strMetadata] = DefineShapeModel("Moon", charDataRootPath, charBpyRootPath, ...
                charOutputLengthUnits="m", ...
                bInitSphericalHarmonicsGravityData=false);

            testCase.verifyTrue(objShapeModel.hasData());
            testCase.verifyEqual(string(strMetadata.charShapeModelObjPath), string(charObjPath));
            testCase.verifyEqual(max(objShapeModel.dVerticesPos, [], "all"), ...
                1000.0 * max(dVerts, [], "all"), "RelTol", 1e-14);
        end

        function testCspiceShapeLoadFailsClearlyWhenMiceIsUnavailable(testCase)
            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            testCShapeModelSimplifyMesh.removeMicePathEntries_();
            objEnvCleanup = testCShapeModelSimplifyMesh.preserveWorkspaceEnv_(); %#ok<NASGU>
            setenv("WS_SIMGEARS", "");
            setenv("WS_NAVSYS", "");

            testCase.verifyError(@() CShapeModel("cspice", "missing_shape.bds", "km", "m", ...
                true, "missing_shape", true), "CShapeModel:CSPICEUnavailable");
        end

        function testCspiceShapeLoadPrefersWsSimGearsMiceWhenAvailable(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charSimGearsRoot = fullfile(string(fixture.Folder), "simgears_root");
            charNavSysRoot = fullfile(string(fixture.Folder), "navsys_root");
            testCShapeModelSimplifyMesh.writeFakeMiceInstall_(charSimGearsRoot, "FakeMice:FromSimGears");
            testCShapeModelSimplifyMesh.writeFakeMiceInstall_(charNavSysRoot, "FakeMice:FromNavSys");

            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            testCShapeModelSimplifyMesh.removeMicePathEntries_();
            objEnvCleanup = testCShapeModelSimplifyMesh.preserveWorkspaceEnv_(); %#ok<NASGU>
            setenv("WS_SIMGEARS", char(charSimGearsRoot));
            setenv("WS_NAVSYS", char(charNavSysRoot));

            testCase.verifyError(@() CShapeModel("cspice", "fake_shape.bds", "km", "m", ...
                true, "fake_shape", true), "FakeMice:FromSimGears");
            testCase.verifyEqual(string(which("cspice_furnsh")), ...
                string(fullfile(charSimGearsRoot, "mice", "src", "mice", "cspice_furnsh.m")));
        end

        function testCspiceShapeLoadFallsBackToWsNavsysMice(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charNavSysRoot = fullfile(string(fixture.Folder), "navsys_root");
            testCShapeModelSimplifyMesh.writeFakeMiceInstall_(charNavSysRoot, "FakeMice:FromNavSys");

            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            testCShapeModelSimplifyMesh.removeMicePathEntries_();
            objEnvCleanup = testCShapeModelSimplifyMesh.preserveWorkspaceEnv_(); %#ok<NASGU>
            setenv("WS_SIMGEARS", "");
            setenv("WS_NAVSYS", char(charNavSysRoot));

            testCase.verifyError(@() CShapeModel("cspice", "fake_shape.bds", "km", "m", ...
                true, "fake_shape", true), "FakeMice:FromNavSys");
            testCase.verifyEqual(string(which("cspice_furnsh")), ...
                string(fullfile(charNavSysRoot, "mice", "src", "mice", "cspice_furnsh.m")));
        end

        function testDefineShapeModelFromShapeCanUseKilometerInputUnits(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "custom_shape_km.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            objShapeModel = DefineShapeModel("FromShape", "", string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelInputUnits="km", ...
                charShapeModelObjPath=string(charObjPath), ...
                bInitSphericalHarmonicsGravityData=false);

            testCase.verifyEqual(max(objShapeModel.dVerticesPos, [], "all"), ...
                1000.0 * max(dVerts, [], "all"), "RelTol", 1e-14);
        end

        function testDefineShapeModelFromShapeRequiresPhysicalInputsForDefaultSH(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "custom_shape_no_physics.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            testCase.verifyError(@() DefineShapeModel("FromShape", "", string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath)), ...
                "DefineShapeModel:MissingPhysicalInputs");
        end

        function testDefineShapeModelFromShapeMassDerivesMetadataAndSH(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "custom_shape_mass.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);
            dMass_kg = 1.0e12;
            dGravConst = 6.67430e-11;

            [objShapeModel, ~, strMetadata] = DefineShapeModel("FromShape", "", string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                dMass_kg=dMass_kg, ...
                ui32SphericalHarmonicsGravityMaxDegree=uint32(2), ...
                ui32SphericalHarmonicsGravityMaxFitIterations=uint32(1));

            strPhysicalMetadata = strMetadata.strPhysicalMetadata;
            strSHgravityData = objShapeModel.getSphericalHarmonicsGravityData();

            testCase.verifyTrue(strPhysicalMetadata.bHasPhysicalMetadata);
            testCase.verifyTrue(strPhysicalMetadata.bHasSphericalHarmonicsGravityData);
            testCase.verifyEqual(strPhysicalMetadata.dMass_kg, dMass_kg, "RelTol", 1e-14);
            testCase.verifyEqual(strPhysicalMetadata.dGravParam_m3mps2, dGravConst * dMass_kg, "RelTol", 1e-14);
            testCase.verifyGreaterThan(strPhysicalMetadata.dVolume_m3, 0.0);
            testCase.verifyEqual(strPhysicalMetadata.dDensity_kgm3, dMass_kg / strPhysicalMetadata.dVolume_m3, "RelTol", 1e-12);
            testCase.verifyEqual(strSHgravityData.dGravParam, strPhysicalMetadata.dGravParam_m3mps2, "RelTol", 1e-14);
            testCase.verifyEqual(strSHgravityData.dDensity, strPhysicalMetadata.dDensity_kgm3, "RelTol", 1e-12);
        end

        function testDefineShapeModelFromShapeRejectsInconsistentPhysicalInputs(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "custom_shape_bad_physics.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            testCase.verifyError(@() DefineShapeModel("FromShape", "", string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                dMass_kg=10.0, ...
                dDensity_kgm3=1.0, ...
                dVolume_m3=1.0), ...
                "BuildShapeModelPhysicalMetadata:InconsistentPhysicalInputs");
        end

        function testDefineShapeModelMetadataOnlyScenarioRequiresNoShapeSource(testCase)
            [objShapeModel, ~, strMetadata] = DefineShapeModel("Earth", "", "", ...
                bLoadShapeModel=false, ...
                bInitSphericalHarmonicsGravityData=false);

            testCase.verifyFalse(objShapeModel.hasData());
            testCase.verifyEqual(strMetadata.enumScenarioName, EnumScenarioName.Earth);
            testCase.verifyGreaterThan(objShapeModel.dObjectReferenceSize, 0.0);
            testCase.verifyError(@() DefineShapeModel("Earth", "", "", ...
                bInitSphericalHarmonicsGravityData=false), ...
                "DefineShapeModel:NoShapeSource");
        end

        function testDefineShapeModelNewTaggedScenariosSupportMetadataOnly(testCase)
            cellScenarioNames = {"Eros", "Arrokoth", "67P", "Toutatis"};

            for idxScenario = 1:numel(cellScenarioNames)
                [objShapeModel, ~, strMetadata] = DefineShapeModel(cellScenarioNames{idxScenario}, "", "", ...
                    bLoadShapeModel=false, ...
                    bInitSphericalHarmonicsGravityData=false);

                testCase.verifyFalse(objShapeModel.hasData());
                testCase.verifyGreaterThan(objShapeModel.dObjectReferenceSize, 0.0);
                testCase.verifyNotEmpty(strMetadata.strScenarioSpec.charDataManifestRelativePath);
                testCase.verifyTrue(any(strcmp(string(strMetadata.strScenarioSpec.cellTags), "shape_runnable")));
            end
        end

        function testDefineShapeModelMissingManifestShapeFailsEarly(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);

            try
                DefineShapeModel("Arrokoth", string(fixture.Folder), "", ...
                    bInitSphericalHarmonicsGravityData=false);
                testCase.verifyTrue(false, "Expected missing shape asset error.");
            catch objException
                testCase.verifyEqual(string(objException.identifier), "DefineShapeModel:MissingShapeAsset");
                charMessage = string(objException.message);
                testCase.verifyTrue(contains(charMessage, "Arrokoth"));
                testCase.verifyTrue(contains(charMessage, "pds_new_horizons_arrokoth_obj"));
                testCase.verifyTrue(contains(charMessage, "mu69_fr2kf_hipoly.obj"));
                testCase.verifyTrue(contains(charMessage, "scenarios/Arrokoth/manifest.json"));
                testCase.verifyTrue(contains(charMessage, "python3 tools/data/fetch_scenario_assets.py"));
                testCase.verifyTrue(contains(charMessage, "--scenario Arrokoth"));
                testCase.verifyTrue(contains(charMessage, "--asset-id pds_new_horizons_arrokoth_obj"));
            end
        end

        function testDefineShapeModelUsesExplicitApophisElongatedScenario(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "apophis_elongated_shape.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            [objApophis, strBpyPaths, strApophisMetadata] = DefineShapeModel(EnumScenarioName.ApophisElongated, ...
                string(fixture.Folder), string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                bInitSphericalHarmonicsGravityData=false);

            testCase.verifyTrue(objApophis.hasData());
            testCase.verifyEqual(string(objApophis.charModelName), "ApophisElongated");
            testCase.verifyEqual(strApophisMetadata.enumScenarioName, EnumScenarioName.ApophisElongated);
            testCase.verifyEqual(string(strApophisMetadata.charCanonicalTargetName), "ApophisElongated");
            testCase.verifyTrue(contains(string(strBpyPaths.charBlenderModelPath), "Elongated"));
        end

        function testDefineShapeModelRejectsLegacySwitchAndItokawaModified(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charObjPath = fullfile(string(fixture.Folder), "removed_legacy_shape.obj");
            [ui32Faces, dVerts] = testCShapeModelSimplifyMesh.buildIcosphere_(uint32(1));
            testCShapeModelSimplifyMesh.writeObjFile_(charObjPath, ui32Faces, dVerts);

            testCase.verifyError(@() DefineShapeModel("ItokawaModified", ...
                string(fixture.Folder), string(fixture.Folder), ...
                charOutputLengthUnits="m", ...
                charShapeModelObjPath=string(charObjPath), ...
                bInitSphericalHarmonicsGravityData=false), ...
                "CScenarioRegistry:UnsupportedScenario");

            try
                DefineShapeModel("Apophis", string(fixture.Folder), string(fixture.Folder), ...
                    charOutputLengthUnits="m", ...
                    charShapeModelObjPath=string(charObjPath), ...
                    bLoadModifiedVariant=true, ...
                    bInitSphericalHarmonicsGravityData=false);
                testCase.verifyFail("bLoadModifiedVariant should not be accepted.");
            catch objException
                testCase.verifyTrue(contains(string(objException.message), "bLoadModifiedVariant") || ...
                    contains(string(objException.message), "Unrecognized") || ...
                    contains(string(objException.message), "Invalid"), ...
                    string(objException.message));
            end
        end
    end

    methods (Static, Access = private)
        function objCleanup = preserveWorkspaceEnv_()
            charOriginalSimGears = getenv("WS_SIMGEARS");
            charOriginalNavsys = getenv("WS_NAVSYS");
            objCleanup = onCleanup(@() testCShapeModelSimplifyMesh.restoreWorkspaceEnv_( ...
                charOriginalSimGears, charOriginalNavsys));
        end

        function restoreWorkspaceEnv_(charOriginalSimGears, charOriginalNavsys)
            setenv("WS_SIMGEARS", charOriginalSimGears);
            setenv("WS_NAVSYS", charOriginalNavsys);
        end

        function removeMicePathEntries_()
            cellPathEntries = string(strsplit(path, pathsep));
            for idxPath = 1:numel(cellPathEntries)
                charPathEntry = cellPathEntries(idxPath);
                if contains(charPathEntry, filesep + "mice" + filesep + "src" + filesep + "mice") || ...
                        contains(charPathEntry, filesep + "mice" + filesep + "lib")
                    rmpath(charPathEntry);
                end
            end
        end

        function writeFakeMiceInstall_(charWorkspaceRoot, charErrorId)
            charMiceSrcPath = fullfile(string(charWorkspaceRoot), "mice", "src", "mice");
            charMiceLibPath = fullfile(string(charWorkspaceRoot), "mice", "lib");
            mkdir(charMiceSrcPath);
            mkdir(charMiceLibPath);

            fileId = fopen(fullfile(charMiceSrcPath, "cspice_furnsh.m"), "w");
            assert(fileId ~= -1, "testCShapeModelSimplifyMesh:FileOpenFailed", ...
                "Failed to create fake cspice_furnsh.m");
            objCleanup = onCleanup(@() fclose(fileId));
            fprintf(fileId, "function cspice_furnsh(varargin)\n");
            fprintf(fileId, "error('%s', 'fake MICE reached');\n", charErrorId);
            fprintf(fileId, "end\n");
            clear objCleanup
        end

        function [ui32Faces, dVerts] = buildIcosphere_(nSubdivisions)
            arguments
                nSubdivisions (1,1) uint32 = uint32(2)
            end

            dPhi = (1 + sqrt(5)) / 2;
            dVerts = [-1  dPhi 0;  1  dPhi 0; -1 -dPhi 0;  1 -dPhi 0;
                       0 -1  dPhi;  0  1  dPhi;  0 -1 -dPhi; 0  1 -dPhi;
                       dPhi 0 -1;  dPhi 0  1; -dPhi 0 -1; -dPhi 0  1];
            dVerts = dVerts ./ vecnorm(dVerts, 2, 2);

            ui32Faces = uint32([ ...
                1 12 6; 1 6 2; 1 2 8; 1 8 11; 1 11 12;
                2 6 10; 6 12 5; 12 11 3; 11 8 7; 8 2 9;
                4 10 5; 4 5 3; 4 3 7; 4 7 9; 4 9 10;
                5 10 6; 3 5 12; 7 3 11; 9 7 8; 10 9 2]);

            for idxSub = 1:nSubdivisions
                nFaces = size(ui32Faces, 1);
                ui32NewFaces = zeros(nFaces * 4, 3, 'uint32');
                edgeMap = containers.Map('KeyType', 'char', 'ValueType', 'uint32');

                for idFace = 1:nFaces
                    ui32FaceVerts = ui32Faces(idFace, :);
                    ui32MidVerts = zeros(1, 3, 'uint32');
                    ui32EdgePairs = [ui32FaceVerts(1) ui32FaceVerts(2); ...
                                     ui32FaceVerts(2) ui32FaceVerts(3); ...
                                     ui32FaceVerts(3) ui32FaceVerts(1)];

                    for idEdge = 1:3
                        charKey = sprintf('%u_%u', ...
                            min(ui32EdgePairs(idEdge, :)), max(ui32EdgePairs(idEdge, :)));
                        if isKey(edgeMap, charKey)
                            ui32MidVerts(idEdge) = edgeMap(charKey);
                        else
                            dNewVert = (dVerts(ui32EdgePairs(idEdge, 1), :) + ...
                                        dVerts(ui32EdgePairs(idEdge, 2), :)) / 2;
                            dNewVert = dNewVert / norm(dNewVert);
                            dVerts = [dVerts; dNewVert]; %#ok<AGROW>
                            ui32MidVerts(idEdge) = uint32(size(dVerts, 1));
                            edgeMap(charKey) = ui32MidVerts(idEdge);
                        end
                    end

                    a = ui32FaceVerts(1);
                    b = ui32FaceVerts(2);
                    c = ui32FaceVerts(3);
                    ab = ui32MidVerts(1);
                    bc = ui32MidVerts(2);
                    ca = ui32MidVerts(3);

                    ui32NewFaces((idFace-1)*4+1, :) = [a  ab ca];
                    ui32NewFaces((idFace-1)*4+2, :) = [b  bc ab];
                    ui32NewFaces((idFace-1)*4+3, :) = [c  ca bc];
                    ui32NewFaces((idFace-1)*4+4, :) = [ab bc ca];
                end

                ui32Faces = ui32NewFaces;
            end
        end

        function writeObjFile_(charObjPath, ui32Faces, dVerts)
            fid = fopen(char(charObjPath), "w");
            assert(fid ~= -1, "testCShapeModelSimplifyMesh:FileOpenFailed", ...
                "Failed to create temporary OBJ file: %s", char(charObjPath));
            cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

            for idVert = 1:size(dVerts, 1)
                fprintf(fid, "v %.16g %.16g %.16g\n", dVerts(idVert, 1), dVerts(idVert, 2), dVerts(idVert, 3));
            end

            for idFace = 1:size(ui32Faces, 1)
                fprintf(fid, "f %u %u %u\n", ui32Faces(idFace, 1), ui32Faces(idFace, 2), ui32Faces(idFace, 3));
            end
        end
    end
end
