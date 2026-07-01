classdef testDefineShapeModel < matlab.unittest.TestCase
    methods (Test)
        function testUsesRegistryReferenceDefaults(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            charOriginalPath = path;
            objPathCleanup = onCleanup(@() path(charOriginalPath)); %#ok<NASGU>
            charFakeCspicePath = fullfile(string(fixture.Folder), "fake_cspice");
            testDefineShapeModel.writeFakeCspiceBodvrd_(charFakeCspicePath);
            addpath(charFakeCspicePath, "-begin");

            cellScenarioNames = {"Apophis", "ApophisElongated", "Itokawa", "Bennu", "Moon", "Mars", "Ceres"};

            for idxScenario = 1:numel(cellScenarioNames)
                charScenarioName = cellScenarioNames{idxScenario};
                [objShapeModel, strBpyPaths, strMetadata] = DefineShapeModel(charScenarioName, "", string(fixture.Folder), ...
                    bLoadShapeModel=false, ...
                    bInitSphericalHarmonicsGravityData=false);
                strScenarioSpec = CScenarioRegistry.GetScenarioSpec(charScenarioName, charLengthUnits="m");

                testCase.verifyEqual(objShapeModel.dObjectReferenceSize, ...
                    strScenarioSpec.dShapeReferenceSize, RelTol=1e-14);
                testCase.verifyEqual(objShapeModel.dTargetShapeMatrix_OF, ...
                    strScenarioSpec.dTargetShapeMatrix_OF, RelTol=1e-14);

                if strlength(strScenarioSpec.charDefaultBlenderRelativePath) > 0
                    charExpectedBlenderPath = fullfile(string(fixture.Folder), strScenarioSpec.charDefaultBlenderRelativePath);
                    testCase.verifyEqual(string(strBpyPaths.charBlenderModelPath), charExpectedBlenderPath);
                    testCase.verifyEqual(string(strMetadata.charBlenderModelPath), charExpectedBlenderPath);
                end
            end
        end
    end

    methods (Static, Access = private)
        function writeFakeCspiceBodvrd_(charFolder)
            mkdir(charFolder);
            fileId = fopen(fullfile(string(charFolder), "cspice_bodvrd.m"), "w");
            assert(fileId ~= -1, "testDefineShapeModel:FileOpenFailed", ...
                "Failed to create fake cspice_bodvrd.m");
            objCleanup = onCleanup(@() fclose(fileId));
            fprintf(fileId, "function dRadii = cspice_bodvrd(varargin)\n");
            fprintf(fileId, "dRadii = [999.0; 999.0; 999.0];\n");
            fprintf(fileId, "end\n");
            clear objCleanup
        end
    end
end
