classdef testCBaseDatastructImport < matlab.unittest.TestCase
    %testCBaseDatastructImport
    % Covers:
    %  - YAML parsing producing cells/cells-of-cells
    %  - ConvertCellsToMatrices_ normalization (vectors/matrices/tensors)
    %  - Wrapper unwrapping (obj<ClassName>)
    %  - fromYaml strict load into CTestStruct (with a minimal YAML that matches its props)

    properties
        objSampleHelper
        objSampleHelperPath
        charTestSamplePath
    end

    methods (TestClassSetup)
        function saveSample(self)
            charThisClassPath = fileparts(mfilename('fullpath'));
            self.charTestSamplePath = fullfile(charThisClassPath, 'tmp_test_samples');
            mkdir(self.charTestSamplePath);

            addpath(fullfile(charThisClassPath, '../test_helpers'));

            self.objSampleHelper = CBaseDatastructTestHelper();
            self.objSampleHelperPath = fullfile(self.charTestSamplePath, 'baseDatastruct_test_sample');

            self.objSampleHelper.saveDataToFile(self.objSampleHelperPath, 'yaml');
            self.objSampleHelper.saveDataToFile(self.objSampleHelperPath, 'struct');
        end
    end

    methods (Test)

        function test_fromYaml(self)
            % Test for constructing a CBaseDatastruct object from corresponding yaml
            objHelper = CBaseDatastructTestHelper();
            objHelper = objHelper.fromYaml(strcat(self.objSampleHelperPath, '.yml'));

            % Check all fields are corresponding to the saved struct
            self.verifyCBaseDatastructMatches(objHelper, self.objSampleHelper, 1e-12);
            
        end

        function test_fromStruct(self)
            % Test for constructing a CBaseDatastruct object from corresponding struct
            strData = load(strcat(self.objSampleHelperPath, '.mat')).(strcat('obj', class(self.objSampleHelper)));
            objHelper = CBaseDatastructTestHelper();
            objHelper = objHelper.fromStruct(strData);

            % Check all fields are corresponding to the saved struct
            self.verifyCBaseDatastructMatches(objHelper, self.objSampleHelper, 1e-12);

        end

    end

    methods (Hidden)
        function verifyCBaseDatastructMatches(testCase, obj, ref, tol)
            %VERIFYCBASEDATASTRUCTMATCHES Deep-verify obj properties vs ref struct fields.
            % Usage:
            %   verifyCBaseDatastructMatches(testCase, objHelper, strData)
            %   verifyCBaseDatastructMatches(testCase, objHelper, strData, 1e-12)

            if nargin < 4 || isempty(tol), tol = 1e-12; end

            % Properties to check (object props ↔ struct fields)
            props = { ...
                'dSimpleScalar' ...
                , 'dSimpleVector' ...
                , 'dSimpleMatrix' ...
                , 'cellNestedCellVectors' ...
                , 'dTensor3D' ...
                , 'charStringList' ...
                , 'strStructArray' ...
                , 'strNestedStruct' ...
                };

            for i = 1:numel(props)
                p = props{i};
                got = obj.(p);
                expct = ref.(p);

                % Normalize cross-type string vs char before deep compare
                if isstring(got) || ischar(got) || isstring(expct) || ischar(expct)
                    got   = string(got);
                    expct = string(expct);
                end

                verifyEqualRecursively(testCase, got, expct, tol, p);
            end
        end

        function verifyEqualRecursively(testCase, a, b, tol, path)
            %VERIFYDEEPEQUAL Recursively verify a and b match (types, sizes, values).
            % Handles: numeric/logical (AbsTol), string/char (exact), cell (elementwise),
            % struct (fieldwise), and mixed string/char normalization done by caller.
            if nargin < 4 || isempty(tol), tol = 1e-12; end
            if nargin < 5, path = ''; end

            % Numeric / logical
            if (isnumeric(a) || islogical(a)) && (isnumeric(b) || islogical(b))
                testCase.verifyClass(a, class(a), sprintf('%s: class changed unexpectedly', path)); 
                testCase.verifyTrue(isequal(size(a), size(b)), sprintf('%s: size mismatch', path));
                testCase.verifyEqual(double(a), double(b), 'AbsTol', tol, sprintf('%s: numeric mismatch', path));
                return
            end

            % String array or char (should already be string-cast by caller, but handle anyway)
            if (isstring(a) || ischar(a)) && (isstring(b) || ischar(b))
                a = string(a); b = string(b);
                testCase.verifyTrue(isequal(size(a), size(b)), sprintf('%s: string size mismatch', path));
                testCase.verifyEqual(a, b, sprintf('%s: string mismatch', path));
                return
            end

            % Cells
            if iscell(a) && iscell(b)
                testCase.verifyTrue(isequal(size(a), size(b)), sprintf('%s: cell size mismatch', path));
                for k = 1:numel(a)
                    verifyEqualRecursively(testCase, a{k}, b{k}, tol, sprintf('%s{%d}', path, k));
                end
                return
            end

            % Structs
            if isstruct(a) && isstruct(b)
                fa = sort(string(fieldnames(a)));
                fb = sort(string(fieldnames(b)));
                testCase.verifyEqual(fa, fb, sprintf('%s: struct field set mismatch', path));

                % Compare element-wise if struct arrays
                testCase.verifyTrue(isequal(size(a), size(b)), sprintf('%s: struct array size mismatch', path));
                for idx = 1:numel(a)
                    for f = reshape(fieldnames(a),1,[])
                        fn = f{1};
                        verifyEqualRecursively( ...
                            testCase, a(idx).(fn), b(idx).(fn), tol, sprintf('%s(%d).%s', path, idx, fn));
                    end
                end
                return
            end

            % Objects: compare their public properties if both are objects of same class
            if isobject(a) && isobject(b) && strcmp(class(a), class(b))
                ma = metaclass(a);
                props = ma.PropertyList([ma.PropertyList.Public] & ~[ma.PropertyList.Dependent]);
                for i = 1:numel(props)
                    pn = props(i).Name;
                    verifyEqualRecursively(testCase, a.(pn), b.(pn), tol, sprintf('%s.%s', path, pn));
                end
                return
            end

            % Fallback: exact equality (covers logical scalar, string scalar already handled)
            testCase.verifyTrue(isequaln(a, b), sprintf('%s: values not equal', path));
        end

    end
end
