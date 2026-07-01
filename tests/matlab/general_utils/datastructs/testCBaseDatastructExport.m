classdef testCBaseDatastructExport < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addTestHelpers(testCase)
            charThisFile = mfilename('fullpath');
            charTestsRoot = fullfile(fileparts(charThisFile), '..', '..');
            charHelpersRoot = fullfile(charTestsRoot, 'test_helpers');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(charHelpersRoot));
        end

        function ensureYamlAvailability(testCase)
            if isempty(which('yaml.dumpFile'))
                stubRoot = testCBaseDatastructExport.createYamlStub();
                addpath(char(stubRoot));
                testCase.addTeardown(@() rmpath(char(stubRoot)));
                testCase.addTeardown(@() testCBaseDatastructExport.removeStubFolder(stubRoot));
            end
        end
    end

    methods (Test)
        function testToStructRemovesEmptyFields(testCase)
            obj = CBaseDatastructTestHelper();
            s = obj.toStruct();
            testCase.verifyTrue(isstruct(s));
            testCase.verifyFalse(isfield(s, 'EmptyField'));
            testCase.verifyEqual(s.dSimpleScalar, obj.dSimpleScalar);
            testCase.verifyEqual(string(s.charStringList), string(obj.charStringList));
        end

        function testToJsonRoundTrip(testCase)
            obj = CBaseDatastructTestHelper();
            jsonStr = obj.toJson();
            parsed = jsondecode(jsonStr);
            testCase.verifyEqual(parsed.dSimpleScalar, obj.dSimpleScalar);
            testCase.verifyEqual(string(parsed.charStringList), string(obj.charStringList));
        end

        function testToYamlIncludesFields(testCase)
            obj = CBaseDatastructTestHelper();
            yamlStr = obj.toYaml();
            testCase.verifyTrue(contains(string(yamlStr), "dSimpleScalar"));
            testCase.verifyTrue(contains(string(yamlStr), "charStringList"));
        end

        function testStaticStructMatchesInstance(testCase)
            obj = CBaseDatastructTestHelper();
            testCase.verifyEqual(CBaseDatastruct.toStructStatic(obj), obj.toStruct());
        end

        function testStaticJsonMatchesInstance(testCase)
            obj = CBaseDatastructTestHelper();
            jsonStr = CBaseDatastruct.toJsonStatic(obj);
            parsed = jsondecode(jsonStr);
            testCase.verifyEqual(parsed.dSimpleScalar, obj.dSimpleScalar);
        end

        function testStaticYamlWrapperFlag(testCase)
            obj = CBaseDatastructTestHelper();
            yamlStr = CBaseDatastruct.toYamlStatic(obj, true, false, "stubPayload");
            testCase.verifyTrue(contains(string(yamlStr), "stubPayload"));
        end

        function testSaveDataToFileJson(testCase)
            obj = CBaseDatastructTestHelper();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            basePath = fullfile(string(fixture.Folder), "json_export");
            obj.saveDataToFile(basePath, "json");
            jsonFile = basePath + ".json";
            testCase.verifyTrue(isfile(jsonFile));
            parsed = jsondecode(fileread(jsonFile));
            testCase.verifyEqual(parsed.dSimpleScalar, obj.dSimpleScalar);
        end

        function testSaveDataToFileYaml(testCase)
            obj = CBaseDatastructTestHelper();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            basePath = fullfile(string(fixture.Folder), "yaml_export");
            obj.saveDataToFile(basePath, "yaml");
            yamlFile = basePath + ".yml";
            testCase.verifyTrue(isfile(yamlFile));
            yamlContent = fileread(yamlFile);
            testCase.verifyTrue(contains(string(yamlContent), "dSimpleScalar"));
            testCase.verifyTrue(contains(string(yamlContent), "charStringList"));
        end

        function testSaveDataToFileStaticJson(testCase)
            obj = CBaseDatastructTestHelper();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            jsonPath = fullfile(string(fixture.Folder), "static.json");
            CBaseDatastruct.saveDataToFileStatic(obj, jsonPath, "json", class(obj));
            testCase.verifyTrue(isfile(jsonPath));
            parsed = jsondecode(fileread(jsonPath));
            testCase.verifyEqual(parsed.dSimpleScalar, obj.dSimpleScalar);
        end

        function testNestedNonBaseObjectIsOmittedFromStruct(testCase)
            strInput = struct();
            strInput.dValue = 1.0;
            strInput.objRuntimePayload = CNonBaseSerializationProbe();

            strOutput = CBaseDatastruct.toStructStatic(strInput);

            testCase.verifyEqual(strOutput.dValue, 1.0);
            testCase.verifyFalse(isfield(strOutput, 'objRuntimePayload'));
        end

        function testNestedUnsupportedRuntimePayloadsAreOmittedBeforeYaml(testCase)
            objMapPayload = containers.Map({'one'}, {1});

            strInput = struct();
            strInput.dValue = 2.0;
            strInput.tablePayload = table([1; 2], [3; 4], 'VariableNames', {'A', 'B'});
            strInput.timetablePayload = timetable(seconds([1; 2]), [5; 6], 'VariableNames', {'A'});
            strInput.mapPayload = objMapPayload;

            strOutput = CBaseDatastruct.toStructStatic(strInput);
            yamlStr = CBaseDatastruct.toYamlStatic(strInput, false, false, "payload");

            testCase.verifyEqual(strOutput.dValue, 2.0);
            testCase.verifyFalse(isfield(strOutput, 'tablePayload'));
            testCase.verifyFalse(isfield(strOutput, 'timetablePayload'));
            testCase.verifyFalse(isfield(strOutput, 'mapPayload'));
            testCase.verifyTrue(contains(string(yamlStr), "dValue"));
            testCase.verifyFalse(contains(string(yamlStr), "tablePayload"));
            testCase.verifyFalse(contains(string(yamlStr), "timetablePayload"));
            testCase.verifyFalse(contains(string(yamlStr), "mapPayload"));
        end

        function testNestedBaseDatastructAndEnumStillSerialize(testCase)
            strInput = struct();
            strInput.objNestedData = CBaseDatastructTestHelper();
            strInput.enumFrameName = EnumFrameName.IN;

            strOutput = CBaseDatastruct.toStructStatic(strInput);

            testCase.verifyTrue(isfield(strOutput, 'objNestedData'));
            testCase.verifyEqual(strOutput.objNestedData.dSimpleScalar, strInput.objNestedData.dSimpleScalar);
            testCase.verifyEqual(string(strOutput.enumFrameName), "IN");
        end

        function testTopLevelNonBaseObjectIsRejected(testCase)
            objRuntimePayload = CNonBaseSerializationProbe();

            testCase.verifyError(@() CBaseDatastruct.toStructStatic(objRuntimePayload), ...
                                 'CBaseDatastruct:InvalidRootType');
        end
    end

    methods (Static, Access = private)
        function stubRoot = createYamlStub()
            stubRoot = string(tempname);
            mkdir(stubRoot);
            pkgFolder = fullfile(stubRoot, "+yaml");
            mkdir(pkgFolder);
            testCBaseDatastructExport.writeStubFunction(fullfile(pkgFolder, "dump.m"), [
                "function charOut = dump(data, varargin)"
                "charOut = jsonencode(data);"
                "if isa(charOut, ""string"")"
                "    charOut = char(charOut);"
                "end"
                "end"
            ]);
            testCBaseDatastructExport.writeStubFunction(fullfile(pkgFolder, "dumpFile.m"), [
                "function dumpFile(filePath, data, varargin)"
                "if nargin < 2"
                "    error('yaml:dumpFile:NotEnoughInputs','Missing data argument.');"
                "end"
                "charOut = jsonencode(data);"
                "fid = fopen(filePath, 'w');"
                "fwrite(fid, charOut, 'char');"
                "fclose(fid);"
                "end"
            ]);
        end

        function writeStubFunction(filePath, contents)
            if isstring(contents)
                contents = strjoin(contents, newline);
            end
            fid = fopen(char(filePath), 'w');
            assert(fid ~= -1, 'Failed to create YAML stub function.');
            cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, '%s', contents);
        end

        function removeStubFolder(folderPath)
            folderPath = char(folderPath);
            if isfolder(folderPath)
                rmdir(folderPath, 's');
            end
        end
    end
end
