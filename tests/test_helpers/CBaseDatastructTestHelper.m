classdef CBaseDatastructTestHelper < CBaseDatastruct
    %% CTestStruct
    % Helper class for unit tests; properties mirror keys in the YAML sample
    % currently embedded in this canvas (SimpleScalar, SimpleVector, ...).
    %
    % Notes on types & your conventions:
    % - d*   -> double (scalars, vectors, matrices, tensors)
    % - b*   -> logical
    % - char*-> strings/chars
    % - str* -> structs / struct arrays
    %
    % Shapes are documented but not enforced at property level; the loader
    % (fromYaml/fromStruct + ConvertCellsToMatrices_) will produce the
    % expected shapes. You can add per-class validation in a setup() method
    % or via explicit setter methods if desired.

    properties (SetAccess = public, GetAccess = public)
        % --- Scalars / vectors / matrices / tensors ---
        dSimpleScalar double = 0.0;        
        dSimpleVector double = rand(1,3);  
        dSimpleMatrix double = rand(4,3);  
        cellNestedCellVectors cell = {rand(3,1), rand(4,6)};  
        dTensor3D double = rand(2,4,5);           

        % --- Strings / text ---
        % Defaulting to a scalar string; loader may expand to string array
        charStringList string = "";     % (1xN string array expected after load)

        % --- Struct arrays ---
        % Homogeneous schema expected: field1 (double), field2 (char/string)
        strStructArray struct = struct('field1', 4.5, 'field2', 'this_is_a_test');

        % --- Nested struct with matrices ---
        strNestedStruct struct = struct('strField', struct('b_value', 84, 'sub2', "ciao"), 'a_value', 42);
    end

    methods
        function self = CBaseDatastructTestHelper(init)
            arguments
                init = []
            end
            if nargin == 1 || isempty(init)
                return; % defaults only
            end

            if isstruct(init)
                self.fromStruct(init, false);
                return;
            end

            if isstring(init) || ischar(init)
                init = string(init);
                % Heuristic: treat as file path if it exists, else YAML content
                if isfile(init)
                    self.fromYaml(init, true, false);
                else
                    self.fromYaml(init, false, false);
                end
                return;
            end
        end
    end
end
