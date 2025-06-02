classdef (Abstract) CBaseDatastruct < handle
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
        
        % "From" methods (these are actually factory methods!)
        function self = fromStruct(self)
            % TODO
            % Sort of copy constructor from struct
        end

        function [charYamlString] = fromYaml(self)
            % TODO, requires yaml package. Re-use code from operative dataset generation
        end

        function [charYamlString] = fromJson(self)
            % TODO, use JSON (MATLAB)
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

            [charRootFolder, ~, charFileExt] = fileparts(charFilename);

            if not(exist(charRootFolder, "dir"))
                mkdir(charRootFolder)
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
                        charFilename = strcat(charFilename, '.yaml');
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


    methods (Static, Access = public)

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
    end


    methods (Static, Access = public)
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
                tmpFieldValue   = strInputStruct.(charFieldName);

                if isstruct(tmpFieldValue)
                    % Recursive cleaning for nested struct
                    strCleanedStruct = CleanAndSortStructFields(tmpFieldValue);
                    % Remove field if nested struct is empty
                    if isempty(fieldnames(strCleanedStruct))
                        strOutputStruct = rmfield(strOutputStruct, charFieldName);
                    else
                        strOutputStruct.(charFieldName) = strCleanedStruct;
                    end

                elseif isempty(tmpFieldValue)
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
                outValue = cellfun(@CBaseDatastruct.convertValue, inVal, 'UniformOutput',false);
 
            elseif isstruct(inVal)
                % Recurse on STRUCTS
                subflds = fieldnames(inVal);

                for idj = 1:numel(subflds)
                    charFieldName = subflds{idj};
                    inVal.(charFieldName) = CBaseDatastruct.convertValue(inVal.(charFieldName));
                end

                outValue = inVal;
            else
                % EVERYTHING ELSE (numeric, char, etc) --> do nothing
                outValue = inVal;
            end
        end

    end

end

