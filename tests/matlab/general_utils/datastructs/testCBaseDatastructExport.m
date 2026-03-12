classdef testCBaseDatastructExport < matlab.unittest.TestCase
    methods (TestClassSetup)
        function ensureYamlAvailability(testCase)
            if isempty(which('yaml.dumpFile'))
                stubRoot = CBaseDatastructExportTests.createYamlStub();
                addpath(char(stubRoot));
                testCase.addTeardown(@() rmpath(char(stubRoot)));
                testCase.addTeardown(@() CBaseDatastructExportTests.removeStubFolder(stubRoot));
            end
        end
    end

    methods (Test)
        function testToStructRemovesEmptyFields(testCase)
            obj = CBaseDatastructTestHelper();
            s = obj.toStruct();
            testCase.verifyTrue(isstruct(s));
            testCase.verifyFalse(isfield(s, 'EmptyField'));
            testCase.verifyEqual(s.Value, obj.Value);
            testCase.verifyEqual(string(s.Name), string(obj.Name));
        end

        function testToJsonRoundTrip(testCase)
            obj = CBaseDatastructTestHelper();
            jsonStr = obj.toJson();
            parsed = jsondecode(jsonStr);
            testCase.verifyEqual(parsed.Value, obj.Value);
            testCase.verifyEqual(string(parsed.Name), string(obj.Name));
        end

        function testToYamlIncludesFields(testCase)
            obj = CBaseDatastructTestHelper();
            yamlStr = obj.toYaml();
            testCase.verifyTrue(contains(string(yamlStr), "Value"));
            testCase.verifyTrue(contains(string(yamlStr), "Name"));
        end

        function testStaticStructMatchesInstance(testCase)
            obj = CBaseDatastructTestHelper();
            testCase.verifyEqual(CBaseDatastruct.toStructStatic(obj), obj.toStruct());
        end

        function testStaticJsonMatchesInstance(testCase)
            obj = CBaseDatastructTestHelper();
            jsonStr = CBaseDatastruct.toJsonStatic(obj);
            parsed = jsondecode(jsonStr);
            testCase.verifyEqual(parsed.Value, obj.Value);
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
            testCase.verifyEqual(parsed.Value, obj.Value);
        end

        function testSaveDataToFileYaml(testCase)
            obj = CBaseDatastructTestHelper();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            basePath = fullfile(string(fixture.Folder), "yaml_export");
            obj.saveDataToFile(basePath, "yaml");
            yamlFile = basePath + ".yml";
            testCase.verifyTrue(isfile(yamlFile));
            yamlContent = fileread(yamlFile);
            testCase.verifyTrue(contains(string(yamlContent), "Value"));
            testCase.verifyTrue(contains(string(yamlContent), "Name"));
        end

        function testSaveDataToFileStaticJson(testCase)
            obj = CBaseDatastructTestHelper();
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            jsonPath = fullfile(string(fixture.Folder), "static.json");
            CBaseDatastruct.saveDataToFileStatic(obj, jsonPath, "json", class(obj));
            testCase.verifyTrue(isfile(jsonPath));
            parsed = jsondecode(fileread(jsonPath));
            testCase.verifyEqual(parsed.Value, obj.Value);
        end
    end

    methods (Static, Access = private)
        function stubRoot = createYamlStub()
            stubRoot = string(tempname);
            mkdir(stubRoot);
            pkgFolder = fullfile(stubRoot, "+yaml");
            mkdir(pkgFolder);
            CBaseDatastructExportTests.writeStubFunction(fullfile(pkgFolder, "dump.m"), [
                "function charOut = dump(data, varargin)"
                "charOut = jsonencode(data);"
                "if isa(charOut, ""string"")"
                "    charOut = char(charOut);"
                "end"
                "end"
            ]);
            CBaseDatastructExportTests.writeStubFunction(fullfile(pkgFolder, "dumpFile.m"), [
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
