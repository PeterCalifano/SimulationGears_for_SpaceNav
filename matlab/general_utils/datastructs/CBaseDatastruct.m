classdef (Abstract) CBaseDatastruct < handle & matlab.mixin.Copyable
    %% DESCRIPTION
    % Base class for datastructs with import/export methods to struct, yaml, json and mat file.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 01-02-2025    Pietro Califano     First placeholder implementation
    % 13-02-2025    Pietro Califano     First implementation of save method to struct
    % 17-02-2025    Pietro Califano     Behaviour change: default is now to save object in mat file directly
    % 01-05-2025    Pietro Califano     Complete prototype implementation of methods to dump to different
    %                                   formats and recursively convert object to struct
    % 02-06-2025    Pietro Califano     Upgrade to remove empty fields when dumping to struct, json, yaml;
    %                                   fix yml dump pipeline, add warning suppression
    % 25-08-2025    Pietro Califano     [MAJOR] Implement prototype methods "from" struct and yaml.
    %                                   Development of release version still open.
    % 27-08-2025    Pietro Califano     Minor improvement to avoid unnecessary warning.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties (SetAccess = public, GetAccess = public)
        bDefaultConstructed {islogical, isscalar} = true;
        charDataHash;
        charInstanceName;
    end

    methods (Access = public)
        % CONSTRUCTOR
        function self = CBaseDatastruct()
            arguments
                
            end
        
        end
            
        % GETTERS
        % TODO complete implementation
        function dataValue = getDataMember(self, charMemberNames)
            arguments
                self
                charMemberNames
            end

            dataValue = cell(length(charMemberNames), 1);
            for idx = 1:length(charMemberNames)

                % TODO: check if field exists
                dataValue{idx} = self.(charMemberNames{idx});
            end
        end
        % TBD: is the method below really useful if attributes are getaccess public? I don't think so
        % function [argout] = queryDataAtID(self, ui32TimeID, charField)
        %     % TODO: method to query a field at a given time ID in time grid
        % 
        % end

        % SETTERS
        function setDataMember(self, charMemberNames, dataValues)
            arguments
                self
                charMemberNames
                dataValues
            end
            for idx = 1:length(charMemberNames)
                % TODO: check if field exists

                self.(charMemberNames{idx}) = dataValues{idx};
            end

        end

        % METHODS
        % Dump "to" methods
        function strData = toStruct(self)
            % Dump to data struct
            strData = CBaseDatastruct.toStructStatic(self);
        end
       
        function [charYamlString] = toYaml(self, bSaveAsWrapped)
            arguments
                self
                bSaveAsWrapped {islogical, isscalar} = false
            end
            % Check yaml package is available
            if isempty( which('yaml.dumpFile') )
                error('YAML toolbox not found. Please install Martin Koch''s yaml and add it to your MATLAB path.');
            end

            % Determine name of saved object
            charClassName = class(self);
            charClassName = strcat("obj",charClassName);

            if bSaveAsWrapped
                % Recursively convert to strOut first
                strTmp.(charClassName) = self.toStruct();
            else
                strTmp = self.toStruct();
            end

            % Emit a YAML string
            charYamlString = yaml.dump(strTmp, "auto");

        end

        function [charJsonString] = toJson(self)

            % Recursively convert to strOut first
            strData = self.toStruct();

            % Parse to JSON
            charJsonString = jsonencode(strData, "PrettyPrint", true);
        
        end
        
        % Factory (instance) methods 
        function self = fromStruct(self, strInput, bStrictUnknown, bStrictMissing)
            %FROMSTRUCT Populate *this* instance from a struct produced by toStruct()/YAML/JSON
            % 1) If a wrapper like struct('obj<ClassName>', <payload>) is provided, unwrap.
            % 2) Validate provided fields vs class properties (public, non-hidden).
            % 3) For each provided field:
            %    - If target property is a CBaseDatastruct (or array/cell thereof), recurse.
            %    - Else, try safe casting to the existing property class (if any default set).
            % 4) For strict mode: error on unexpected fields or missing properties without defaults.
            % 5) Return the populated instance.
            arguments
                self     (1,1) {mustBeA(self, "CBaseDatastruct")}
                strInput (1,1) struct {isstruct}
                bStrictUnknown  (1,1) logical {islogical} = true
                bStrictMissing  (1,1) logical {islogical} = false
            end

            % Unwrap wrapper if present: struct('obj<Class>', payload)
            strData = CBaseDatastruct.unwrapWrapper_(strInput, class(self));

            % Discover this class public properties
            objMeta = metaclass(self);
            cellPropNamesAll = {objMeta.PropertyList.Name};
            % Keep only public, non-dependent, non-constant properties
            bKeep = arrayfun(@(p) strcmp(p.GetAccess,'public') && ~p.Constant && ~p.Dependent, objMeta.PropertyList);

            cellPropNames = cellPropNamesAll(bKeep);
            % Build a set for fast membership tests
            strProvided = strData; %#ok<NASGU>
            cellProvidedNames = fieldnames(strData);

            % Strictness checks: unknown fields
            if bStrictUnknown
                cellUnknown = setdiff(cellProvidedNames, cellPropNames);
                if ~isempty(cellUnknown)
                    error('fromStruct:UnknownFields', 'Unknown fields for class %s: %s', class(self), strjoin(cellUnknown', ', '));
                end
            end

            % Assign known fields
            for idF = 1:numel(cellProvidedNames)
                charFld = cellProvidedNames{idF};
                
                if ismember(charFld, cellPropNames)
                    self = CBaseDatastruct.assignField_(self, charFld, strData.(charFld), bStrictUnknown);
                else
                    % Ignore silently (or warn) if non-strict
                    % warning('fromStruct:IgnoringField','Ignoring unknown field %s for class %s.', charFld, class(self));
                end
            end

            % Strictness checks: missing fields (heuristic)
            if bStrictMissing
                % Consider a property missing if it has no value and no default.
                % We detect defaults from the current instance state (post-assignment).
                for idp = 1:numel(cellPropNames)
                    charP = cellPropNames{idp};
                    if ~isprop(self, charP) 
                        continue; 
                    end 
                    % Heuristic: treat as missing if isempty and the input struct had no field for it
                    if ~isfield(strData, charP) && isempty(self.(charP))
                        error('fromStruct:MissingField','Missing required field %s for class %s.', charP, class(self));
                    end
                end
            end

            % Set as not default constructed (loaded from data)
            if isprop(self, 'bDefaultConstructed')
                self.bDefaultConstructed = false;
            end
        end

        function self = fromYaml(self, charInputYaml, bIsFile, bStrict)
            % FROMYAML Populate *this* from YAML (string or file), leveraging yaml toolbox.
            % Known limitations: 
            % 1) Cells containing vectors/matrices/arrays with rows/cols matching in size are converted to
            % single vectors/matrixs/arrays. This is because the parser cannot distinguish whether they
            % were dumped from a vector/matrix/array and loaded by yaml as cells of cells or not.

            arguments
                self
                charInputYaml {mustBeText}
                bIsFile  logical {islogical} = []   % Auto-detect if empty
                bStrict (1,1) logical {islogical} = false
            end

            % Check yaml package is available
            if isempty( which('yaml.load') ) && isempty( which('yaml.ReadYaml') )
                error('No YAML toolbox found. Please install Martin Koch''s yaml and add it to your MATLAB path.');
            end

            % Auto-detect file vs string if not specified
            if isempty(bIsFile)
                [~, ~, charExt] = fileparts(charInputYaml);

                if strcmpi(charExt, '.yml') || strcmpi(charExt, '.yaml')
                    bIsFile = true && isfile(string(charInputYaml));

                    if not(bIsFile)
                        error('Correct file extention, but MATLAB cannot find file. Please specify full path or add to path.')
                    end

                elseif isfile(string(charInputYaml))
                    error('Invalid extension %s.', charExt);
                end
            end

            if isempty(bIsFile)
                error('Automatic deduction of bIsFile failed. Cannot determine if input variable is a path or YAML string. Please specify.');
            end

            if bIsFile
                mustBeFile(string(charInputYaml));

                % Prefer yaml.loadFile if available, else fallback
                if ~isempty(which('yaml.loadFile'))
                    strParsed = yaml.loadFile(string(charInputYaml));
                else
                    % Fallback: read text, then parse
                    charText = fileread(string(charInputYaml));
                    if ~isempty(which('yaml.load'))
                        strParsed = yaml.load(charText);
                    else
                        strParsed = yaml.ReadYaml(charText); %#ok<NASGU>
                        error('fromYaml:Compat',['yaml.ReadYaml fallback not wired to struct output here. ' ...
                            'Please ensure yaml.load is available.']);
                    end
                end
            else
                % YAML content as text
                if ~isempty(which('yaml.load'))
                    strParsed = yaml.load(string(charInputYaml));
                else
                    error('fromYaml:NoLoader','yaml.load not found. Use yaml.loadFile or provide a file path.');
                end
            end

            if ~isstruct(strParsed)
                error('fromYaml:ParseError','YAML parse did not yield a struct.');
            end

            % Convert parsed to ensure no numeric vector/matrix field is a cell
            strParsed = CBaseDatastruct.ConvertCellsToMatrices_(strParsed);

            % Call method to build data from yaml-parsed struct
            self = self.fromStruct(strParsed, true);
        end

        function self = fromJson(self, charInputJson, bIsFile, bStrict)
            %FROMJSON Populate *this* from JSON (string or file)
            arguments
                self
                charInputJson {mustBeText}
                bIsFile {mustBeScalarOrEmpty} = []   % Auto-detect if empty
                bStrict (1,1) logical {islogical} = false
            end

            if isempty(bIsFile)
                bIsFile = isfile(string(charInputJson));
            end

            if bIsFile
                mustBeFile(string(charInputJson));
                charText = fileread(string(charInputJson));
            else
                charText = char(charInputJson);
            end

            try
                strParsed = jsondecode(charText);
            catch ME
                error('fromJson:ParseError','JSON decode failed: %s', ME.message);
            end

            if ~isstruct(strParsed)
                error('fromJson:ParseError','JSON parse did not yield a struct.');
            end

            % Call method to build data from yaml-parsed struct
            self = self.fromStruct(strParsed, bStrict);
        end

        function [self] = saveDataToFile(self, charFilename, charFormat)
            arguments
                self
                charFilename    (1,:) string {mustBeA(charFilename, ["string", "char"])} = fullfile('.', lower(string(class(self))));
                charFormat      (1,:) string {mustBeA(charFormat, ["string", "char"]), mustBeMember(charFormat, ["json", "yaml", "mat", "struct", "yml"])} = "mat"
            end

            if charFormat == "yml"
                charFormat = "yaml"; % To allow both formats
            end

            % Object saving method
            fprintf("\nSaving datastruct to file %s in format %s...", charFilename, charFormat);

            [charRootFolder, charFilename_, charFileExt] = fileparts(charFilename);

            if not(exist(charRootFolder, "dir"))
                mkdir(charRootFolder)
            end

            if strcmpi(charRootFolder, '')
                charRootFolder = '.';
                charFilename = fullfile( charRootFolder, strcat(charFilename_, charFileExt) );
            end

            % Determine name of saved object
            charClassName = class(self);
            charClassName = strcat("obj",charClassName);

            % Save according to requested format
            switch charFormat

                case "mat"
                    if isempty(charFileExt) || strcmpi(charFileExt, "")
                        charFilename = strcat(charFilename, '.mat');
                    end
                           
                    strTmp.(charClassName) = self;
                    save(charFilename, "-struct", "strTmp"); % Save content of strTmp
                
                case "struct"
                    if isempty(charFileExt) || strcmpi(charFileExt, "")
                        charFilename = strcat(charFilename, '.mat');
                    end
                    warning(['This may be a lossy saving, as all properties that contain objects may be lost ' ...
                        'if cannot be converted to struct. Prefer using .mat format directly if possible.'])

                    strData = self.toStruct();

                    strTmp.(charClassName) = strData;
                    save(charFilename, "-struct", "strTmp"); % Save content of strTmp

                case "yaml"
                    if isempty(charFileExt) || strcmpi(charFileExt, "")
                        charFilename = strcat(charFilename, '.yml');
                    end
                    
                    % Check yaml package is available
                    if isempty( which('yaml.dumpFile') )
                        error('YAML toolbox not found. Please install Martin Koch''s yaml and add it to your MATLAB path.');
                    end
                    
                    charYamlParsed = self.toYaml();
                    
                    % Write file to disk
                    fileID = fopen(charFilename, 'w');
                    fwrite(fileID, charYamlParsed, 'char');
                    fclose(fileID);

                case "json"
                    if isempty(charFileExt) || strcmpi(charFileExt, "")
                        charFilename = strcat(charFilename, '.json');
                    end

                    charJsonParsed = self.toJson();

                    % Write to file
                    fileID = fopen(charFilename, 'w');
                    fprintf(fileID, charJsonParsed, 'char');
                    fclose(fileID);

                otherwise
                    error('Invalid selected format.')

            end

            fprintf(" DONE.\n");

        end

    end

    %% Static methods
    methods (Static, Access = public)

        function strParsedNoCell = ConvertCellsToMatrices_(strParsed)
            arguments
                strParsed (1,1) {isstruct}
            end

            % Initialize output
            strParsedNoCell = strParsed;

            % Process struct
            cellFieldnames = fieldnames(strParsed);

            for idF = 1:length(cellFieldnames)
                charFieldName = cellFieldnames{idF};

                varFieldVal = strParsed.(charFieldName);

                if isstruct(varFieldVal)
                    % Recursive call for struct
                    strParsedNoCell.(charFieldName) = CBaseDatastruct.ConvertCellsToMatrices_(varFieldVal);

                elseif iscell(varFieldVal)
                    % Handle cell field recursively
                    % try
                        % Attempt to convert to numeric value first
                        % if all(cellfun(@(x) isnumeric(x) || islogical(x), varFieldVal(:)))
                        %     % Flatten nested 1xN cells of numeric/logical into matrix if dimensions align
                        %     % Example: {{1,2,3},{4,5,6}} → [1 2 3; 4 5 6]
                        %     varTmpMat = cellfun(@(x) double(x), varFieldVal);
                        % 
                        %     if isvector(varTmpMat)
                        %         % Assign and continue
                        %         strParsedNoCell.(charFieldName) = varTmpMat;
                        %         continue;
                        %     end
                        % 
                        % elseif all(cellfun(@(x) iscell(x), varFieldVal))
                        %     % Check aligning dimensions, if matching, build matrix
                        %     cellTmpInternalProcessed = CBaseDatastruct.handleCellElement_(varFieldVal);
                        % 
                        % end

                    % Recurse into each cell element, then try to collapse
                    strParsedNoCell.(charFieldName) = CBaseDatastruct.handleCellElement_(varFieldVal);
                    
                    % catch
                    %     % If conversion fails, keep original
                    %     strParsedNoCell.(charFieldName) = varFieldVal;
                    % end
                else
                    % Primitive: assign directly
                    strParsedNoCell.(charFieldName) = varFieldVal;
                end
            end % Endfor on fields
        end

        function outStruct = toStructStatic(objDatastruct)
            arguments
                objDatastruct (:,1)
            end

            % Disable warning temporarily
            warning('off', 'MATLAB:structOnObject');

            % Do shallow conversion first      
            outStruct = struct(objDatastruct);
            
            % Enable warnings again
            warning('on', 'MATLAB:structOnObject');

            % Recursively convert any nested object/cell/struct
            cellFlds = fieldnames(outStruct);

            for idi = 1:numel(cellFlds)
                charName = cellFlds{idi};
                outStruct.(charName) = CBaseDatastruct.convertValue_(outStruct.(charName));
            end

            % Remove bDefaultConstructedField if existing
            if isfield(outStruct, "bDefaultConstructed")
                outStruct = rmfield(outStruct, "bDefaultConstructed");
            end

            % Remove empty fields and order struct
            outStruct = CBaseDatastruct.CleanAndSortStructFields(outStruct);
        end
    

        % TODO ------------------
        function objDatastruct = fromStructStatic(strDatastruct, charTargetDatastruct) % TBC, better if abstract?
            arguments (Input)
                strDatastruct % either yaml string or path to yaml file
                charTargetDatastruct % Specify name of target data struct
            end
            arguments (Output)
                objDatastruct (1,1) {isa(objDatastruct, 'CBaseDatastruct')}
            end

            % Method to convert struct to class (with fields check)

        end

        function [] = fromYamlStatic(charInputFile)
            arguments
                charInputFile % either yaml string or path to yaml file
            end
            % TODO, requires yaml package. Re-use code from operative dataset generation
        end

        function [] = fromJsonStatic(charInputFile)
            arguments
                charInputFile % either json string or path to json file
            end
            % TODO, use JSON (MATLAB)
        end

        % TODO with protobuf or msgpack
        function [] = serialize()

        end

        function [] = deserialize()

        end
   
        function strOutputStruct = CleanAndSortStructFields(strInputStruct)
            %CLEANSORTSTRUCTFIELDS Recursively remove empty fields from a struct and sort fields
            %   strOutputStruct = CLEANSORTSTRUCTFIELDS(strInputStruct) takes a struct strInputStruct
            %   (possibly nested) and returns strOutputStruct where any field whose value is empty is removed.
            %   All nested structs are cleaned recursively, and their fields are sorted alphabetically.

            arguments
                strInputStruct (1,1) struct   % Input struct to clean and sort
            end

            % Initialize output as a copy of the input
            strOutputStruct = strInputStruct;

            % Retrieve all field names (cell array of char)
            cellFieldNames = fieldnames(strInputStruct);

            % Loop through each field using a index
            for dIdx = 1:numel(cellFieldNames)
                charFieldName = cellFieldNames{dIdx};
                varTmpFieldValue   = strInputStruct.(charFieldName);

                if isstruct(varTmpFieldValue)
                    % Recursive cleaning for nested struct
                    strCleanedStruct = CBaseDatastruct.CleanAndSortStructFields(varTmpFieldValue);
                    % Remove field if nested struct is empty
                    if isempty(fieldnames(strCleanedStruct))
                        strOutputStruct = rmfield(strOutputStruct, charFieldName);
                    else
                        strOutputStruct.(charFieldName) = strCleanedStruct;
                    end

                elseif isempty(varTmpFieldValue)
                    % Remove field if its value is empty
                    strOutputStruct = rmfield(strOutputStruct, charFieldName);
                end
            end

            % After cleaning, sort remaining fields alphabetically
            if ~isempty(strOutputStruct) && isstruct(strOutputStruct)
                try
                    % Use MATLAB's built-in orderfields (R2017b+)
                    strOutputStruct = orderfields(strOutputStruct);
                catch
                    % Fallback for older MATLAB versions
                    cellSortedFieldNames = sort(fieldnames(strOutputStruct));
                    strReorderedStruct  = struct();
                    for dIdx2 = 1:numel(cellSortedFieldNames)
                        charSortedName = cellSortedFieldNames{dIdx2};
                        strReorderedStruct.(charSortedName) = strOutputStruct.(charSortedName);
                    end
                    strOutputStruct = strReorderedStruct;
                end
            end
        end

    end

    methods (Static, Access = private)

        function varOut = handleCellElement_(varIn)
            arguments
                varIn
            end

            %HANDLECELLELEMENT_ Helper for ConvertCellsToMatrices_
            % Recursively normalize cells that come from YAML lists into
            % numeric matrices/vectors/tensors when possible.
            if isstruct(varIn)
                varOut = CBaseDatastruct.ConvertCellsToMatrices_(varIn);
                return;
            end

            if ~iscell(varIn)
                % Base case: primitive / already-normalized value
                varOut = varIn;
                return;
            end

            % 1) Recurse element-wise first
            cellRec = cellfun(@(y) CBaseDatastruct.handleCellElement_(y), varIn, 'UniformOutput', false);

            % 2) Struct cells -> struct array if schema matches
            if ~isempty(cellRec) && all(cellfun(@isstruct, cellRec(:)))
                try
                    varOut = reshape([cellRec{:}], size(cellRec));
                    return;
                catch
                    % fall through and keep as cell if concatenation fails
                end
            end

            % 3) All strings/chars -> string array
            if ~isempty(cellRec) && all(cellfun(@(y) isstring(y) || ischar(y), cellRec(:)))
                try
                    varOut = string(cellRec);
                    return;
                catch
                    % keep as cell if shapes disagree
                end
            end

            % 4) All numeric/logical -> attempt dense conversions
            isNumLike = @(y) (isnumeric(y) || islogical(y));
            if ~isempty(cellRec) && all(cellfun(isNumLike, cellRec(:)))
                % 4.a scalars -> cell2mat directly
                if all(cellfun(@isscalar, cellRec(:)))
                    try
                        varOut = cell2mat(cellRec);
                        varOut = reshape(varOut, size(cellRec));
                        return;
                    catch
                        % Class mismatch: cast to double then cell2mat
                        cellCast = cellfun(@double, cellRec, 'UniformOutput', false);
                        varOut = cell2mat(cellCast);
                        return;
                    end
                end

                % 4.b vectors with identical length -> stack into 2-D
                if all(cellfun(@isvector, cellRec(:)))
                    ui32Lengths = cellfun(@numel, cellRec(:));
                    if isscalar(unique(ui32Lengths))
                        % Choose orientation based on first element
                        varFirst = cellRec{1};

                        if isrow(varFirst)
                            % Row vectors -> rows stacked (M x N)
                            varOut = vertcat(cellRec{:});
                        else
                            % Column vectors -> columns stacked (N x M)
                            varOut = horzcat(cellRec{:});
                        end
                        return;
                    end
                end

                % 4.c 2-D matrices with identical size -> 3-D tensor cat along 3rd dim
                bIsMatrix2D = @(y) isnumeric(y) && ismatrix(y);
                if all(cellfun(bIsMatrix2D, cellRec(:)))
                    ui32Size1 = size(cellRec{1});
                    bHasSameSize = all(cellfun(@(y) isequal(size(y), ui32Size1), cellRec(:)));
                    
                    if bHasSameSize
                        try
                            varOut = cat(3, cellRec{:});
                            return;
                        catch
                            % As a fallback, preallocate then assign
                            varOut = zeros([ui32Size1 numel(cellRec)], 'like', cellRec{1});
                            for idk = 1:numel(cellRec)
                                varOut(:,:,idk) = cellRec{idk};
                            end
                            return;
                        end
                    end
                end

            end

            % 5) If none of the above patterns matched, keep as (normalized) cell
            varOut = cellRec;
        end


        function outValue = convertValue_(inVal)
            % Private helper function to recurse fields when converting objects
            if isobject(inVal) && not(isstring(inVal))
                % Recurse on OBJECTS
                if numel(inVal) > 1
                    % Array of objects → struct array
                    tmpConvertedArray = arrayfun(@(o) o.toStruct(), inVal);
                    outValue = reshape(tmpConvertedArray, size(inVal));

                elseif ismethod(inVal, 'toStruct')
                    % If object has method "toStruct" (base is this class, call it)
                    outValue = inVal.toStruct();

                else
                    % Else, fallback to casting
                    outValue = struct(inVal);
                end

            elseif iscell(inVal)
                % Recurse on CELLS
                outValue = cellfun(@CBaseDatastruct.convertValue_, inVal, 'UniformOutput',false);
 
            elseif isstruct(inVal)
                % Recurse on STRUCTS
                subflds = fieldnames(inVal);

                for idj = 1:numel(subflds)
                    charFieldName = subflds{idj};
                    inVal.(charFieldName) = CBaseDatastruct.convertValue_(inVal.(charFieldName));
                end

                outValue = inVal;
            else
                % EVERYTHING ELSE (numeric, char, etc) --> do nothing
                outValue = inVal;
            end
        end

        function strUnwrappedField = unwrapWrapper_(strValueIn, charTargetClass)
            %UNWRAPWRAPPER_ If top-level struct is like struct('obj<Class>', payload), unwrap it.
            arguments
                strValueIn (1,1) struct
                charTargetClass (1,:) char
            end

            if isscalar(fieldnames(strValueIn))

                charFld = fieldnames(strValueIn); charFld = charFld{1};
                charExpected = strcat('obj', charTargetClass);

                if strcmp(charFld, charExpected)
                    strUnwrappedField = strValueIn.(charFld);
                    if ~isstruct(strUnwrappedField)
                        error('unwrapWrapper_:PayloadType', 'Wrapper field %s must contain a struct.', charFld);
                    end
                    return;
                end
            end

            strUnwrappedField = strValueIn;
        end

        function self = assignField_(self, charField, varValueIn, bStrict)
            %ASSIGNFIELD_ Assign one property with type-aware recursion/casting
            % Implementation:
            % 1) Inspect current property default/type via meta.Property and runtime value.
            % 2) If target is a CBaseDatastruct (single, array, or in cell), recurse building objects.
            % 3) Else if numeric/logical/char/string, attempt safe cast to existing class.
            % 4) Else assign verbatim.
            arguments
                self
                charField   (1,:) char
                varValueIn
                bStrict     (1,1) logical {islogical}
            end

            % If the property does not exist (possible non-strict), skip
            if ~isprop(self, charField)
                return;
            end

            % Inspect meta
            objMeta = metaclass(self);
            idx = find(strcmp({objMeta.PropertyList.Name}, charField), 1, 'first');
            objMetaProperties = objMeta.PropertyList(idx);

            % Determine target class expectation from current value (default)
            hasDefault = false; 
            charDefaultClass = '';
            try
                currVal = self.(charField);
                if ~isempty(currVal)
                    hasDefault = true; charDefaultClass = class(currVal);
                end
            catch
                currVal = [];
            end

            % Helper to check subclass-of CBaseDatastruct
            isCBaseSubclass = @(cls) ~isempty(cls) && exist(cls,'class')==8 && any(strcmp(superclasses(cls), 'CBaseDatastruct'));

            % Case A: struct destined to a CBaseDatastruct-like property
            if isstruct(varValueIn)
                % If default value is a CBaseDatastruct, use its class to build
                if hasDefault && isCBaseSubclass(charDefaultClass)
                    newObj = feval(charDefaultClass);
                    newObj = newObj.fromStruct(varValueIn, bStrict);
                    self.(charField) = newObj;
                    return;
                end

                % If no default class info, but property name hints a class? Not reliable.
                % Try soft match if the struct is wrapped (obj<Class>)
                valueMaybeUnwrapped = CBaseDatastruct.unwrapWrapper_(varValueIn, ''); %#ok<NASGU>
                % Without schema knowledge, assign struct as-is.
                self.(charField) = varValueIn;
                return;
            end

            % Case B: struct array -> object array or struct array assignment
            if isstruct(varValueIn) && numel(varValueIn) > 1 
                if hasDefault && isCBaseSubclass(charDefaultClass)
                    arr = arrayfun(@(s) feval(charDefaultClass).fromStruct(s, bStrict), varValueIn, 'UniformOutput', false);
                    % Convert to homogeneous array if possible
                    try
                        self.(charField) = reshape([arr{:}], size(varValueIn));
                    catch
                        self.(charField) = arr; % fallback to cell array
                    end
                else
                    self.(charField) = varValueIn; % keep as struct array
                end
                return;
            end

            % Case C: cell containing structs/objects -> recurse per element
            if iscell(varValueIn)
                cellIn = varValueIn;
                if hasDefault && isCBaseSubclass(charDefaultClass)
                    % Expecting cell of objects; map structs to objects of that class
                    cellOut = cell(size(cellIn));
                    for k = 1:numel(cellIn)
                        vk = cellIn{k};
                        if isstruct(vk)
                            cellOut{k} = feval(charDefaultClass).fromStruct(vk, bStrict);
                        else
                            cellOut{k} = vk; % accept already-built object or scalar
                        end
                    end
                    self.(charField) = cellOut;
                else
                    % Generic cell: just recurse lightly over structs
                    cellOut = cell(size(cellIn));
                    for k = 1:numel(cellIn)
                        % vk = cellIn{k};
                        % if isstruct(vk)
                        cellOut{k} = cellIn{k};
                        % else
                        %     cellOut{k} = vk;
                        % end
                    end
                    self.(charField) = cellOut;
                end
                return;
            end

            % Case D: primitive types — try safe cast to default class if present
            if hasDefault && ~isempty(charDefaultClass)
                try
                    switch charDefaultClass
                        case {'double','single','uint8','uint16','uint32','uint64','int8','int16','int32','int64'}
                            self.(charField) = cast(varValueIn, charDefaultClass);
                            return;
                        case {'logical'}
                            self.(charField) = logical(varValueIn);
                            return;
                        case {'string'}
                            self.(charField) = string(varValueIn);
                            return;
                        case {'char'}
                            if isstring(varValueIn); varValueIn = char(varValueIn); end
                            self.(charField) = char(varValueIn);
                            return;
                        otherwise
                            % If default is an object (non-CBaseDatastruct), accept as-is if compatible
                            if isa(varValueIn, charDefaultClass)
                                self.(charField) = varValueIn; return;
                            end
                    end
                catch ME
                    if bStrict
                        rethrow(ME);
                    else
                        % fallthrough to plain assignment
                    end
                end
            end

            % Default: assign verbatim
            self.(charField) = varValueIn;
        end
    end

end

