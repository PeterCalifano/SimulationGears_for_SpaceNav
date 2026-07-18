classdef SSphericalHarmonicsGravityData < CBaseDatastruct
    %% DESCRIPTION
    % Serializable datastruct for repo-native spherical-harmonics gravity data.
    % -------------------------------------------------------------------------------------------------------------
    %% SAVED DATA CONVENTION
    % Save MAT files with one variable named strSphericalHarmonicsGravityData containing the plain struct
    % returned by toSavedStruct().
    %
    % strDocumentationHeader.charSavedDataConvention must be:
    % SimulationGears.SSphericalHarmonicsGravityData.v01
    %
    % Coefficients are stored in dCSlmCoeffCols as unnormalized [Clm, Slm] column pairs. Row 1 is the
    % degree-1 compatibility row (l,m)=(1,1), which is accepted but ignored by degree >= 2 evaluators.
    % Rows for degree l >= 2 then follow by increasing degree and order m=0..l. Degree L requires
    % (L + 1) * (L + 2) / 2 - 2 rows.
    %
    % dGravParam is stored in charLengthUnits^3/s^2. dBodyRadiusRef is stored in charLengthUnits.
    % dGravConst is not user-tunable: it must match the physical gravitational constant scaled to
    % charLengthUnits.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 30-06-2026    Pietro Califano, Codex     First schema-backed implementation for file-backed SH data.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % saveToFile, fromFile, fromSavedStruct, toSavedStruct, toGravityDataStruct, toMetadataStruct
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % See properties().
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CBaseDatastruct, CScenarioRegistry
    % -------------------------------------------------------------------------------------------------------------

    properties (Constant, Access = private)
        charSavedDataConvention_ (1,:) char = 'SimulationGears.SSphericalHarmonicsGravityData.v01';
        charSavedStructVariable_ (1,:) char = 'strSphericalHarmonicsGravityData';
    end

    properties (SetAccess = public, GetAccess = public)
        strDocumentationHeader (1,1) struct = struct()
        ui32SchemaVersion (1,1) uint32 = uint32(1)
        charSavedDataConvention (1,:) char = 'SimulationGears.SSphericalHarmonicsGravityData.v01'
        charFormat (1,:) char = 'SSphericalHarmonicsGravityData'
        charCoefficientStorage (1,:) char = 'cslm_cols'
        charNormalization (1,:) char = 'unnormalized'
        enumScenarioName (1,1) EnumScenarioName = EnumScenarioName.NotDefined
        charScenarioName (1,:) char = ''
        enumLengthUnits (1,1) EnumLengthUnits = EnumLengthUnits.m
        charLengthUnits (1,:) char = 'm'
        ui32MaxDegree (1,1) uint32 = uint32(0)
        dCSlmCoeffCols (:,2) double = zeros(0, 2)
        dGravParam (1,1) double = NaN
        dBodyRadiusRef (1,1) double = NaN
        dDensity (1,1) double = NaN
        dGravConst (1,1) double = NaN
        charSource (1,:) char = ''
        charSourceUrl (1,:) char = ''
        charFieldKind (1,:) char = 'file'
    end

    methods (Access = public)
        function self = SSphericalHarmonicsGravityData(options)
            arguments
                options.charScenarioName (1,:) {mustBeA(options.charScenarioName, ["string", "char", "EnumScenarioName"])} = ""
                options.charLengthUnits {mustBeA(options.charLengthUnits, ["string", "char", "EnumLengthUnits"])} = "m"
                options.ui32MaxDegree (1,1) uint32 = uint32(0)
                options.dCSlmCoeffCols (:,2) double = zeros(0, 2)
                options.dGravParam (1,1) double = NaN
                options.dBodyRadiusRef (1,1) double = NaN
                options.dDensity (1,1) double = NaN
                options.dGravConst (1,1) double = NaN
                options.charSource (1,:) {mustBeA(options.charSource, ["string", "char"])} = ""
                options.charSourceUrl (1,:) {mustBeA(options.charSourceUrl, ["string", "char"])} = ""
                options.charFieldKind (1,:) {mustBeA(options.charFieldKind, ["string", "char"])} = "file"
            end
            %% SIGNATURE
            % self = SSphericalHarmonicsGravityData(options)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Construct a schema-backed spherical-harmonics gravity data object for a single scenario.
            % Non-empty coefficient data are validated at construction time.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % options.charScenarioName: Scenario enum/name associated with the coefficient data.
            % options.charLengthUnits: Length unit for dGravParam, dBodyRadiusRef, and dGravConst.
            % options.ui32MaxDegree: Maximum spherical-harmonics degree represented by dCSlmCoeffCols.
            % options.dCSlmCoeffCols: Unnormalized [Clm, Slm] coefficient columns.
            % options.dGravParam: Gravitational parameter in charLengthUnits^3/s^2.
            % options.dBodyRadiusRef: Reference radius in charLengthUnits.
            % options.dDensity: Optional bulk density metadata.
            % options.dGravConst: Physical gravitational constant scaled to charLengthUnits.
            % options.charSource: Human-readable data source label.
            % options.charSourceUrl: Source URL or archive locator.
            % options.charFieldKind: Field provenance label, usually file.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % self: Initialized SSphericalHarmonicsGravityData object.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CBaseDatastruct, EnumLengthUnits, CScenarioRegistry
            % -------------------------------------------------------------------------------------------------------------

            self@CBaseDatastruct();

            self.strDocumentationHeader = SSphericalHarmonicsGravityData.BuildDocumentationHeader_();
            [self.enumScenarioName, self.charScenarioName] = ...
                SSphericalHarmonicsGravityData.NormalizeScenario_(options.charScenarioName);
            self.enumLengthUnits = EnumLengthUnits.fromAny(options.charLengthUnits);
            self.charLengthUnits = char(EnumLengthUnits.toString(self.enumLengthUnits));
            self.ui32MaxDegree = options.ui32MaxDegree;
            self.dCSlmCoeffCols = options.dCSlmCoeffCols;
            self.dGravParam = options.dGravParam;
            self.dBodyRadiusRef = options.dBodyRadiusRef;
            self.dDensity = options.dDensity;
            self.charSource = char(options.charSource);
            self.charSourceUrl = char(options.charSourceUrl);
            self.charFieldKind = char(options.charFieldKind);

            if isfinite(options.dGravConst)
                self.dGravConst = options.dGravConst;
            else
                self.dGravConst = SSphericalHarmonicsGravityData.ResolveGravConstForLengthUnit_(self.charLengthUnits);
            end

            if self.ui32MaxDegree > uint32(0) || ~isempty(self.dCSlmCoeffCols)
                self.bDefaultConstructed = false;
                self.validate();
            end
        end

        function validate(self, ui32RequestedDegree, options)
            arguments
                self (1,1) SSphericalHarmonicsGravityData
                ui32RequestedDegree (1,1) uint32 = uint32(0)
                options.strScenarioSpec (1,1) struct = struct()
                options.bRequireScenarioMatch (1,1) logical = false
            end
            %% SIGNATURE
            % validate(self, ui32RequestedDegree, options)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Validate the saved-data schema, coefficient-table shape, physical constants, requested degree,
            % and optional consistency with a scenario registry specification.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self: SSphericalHarmonicsGravityData object to validate.
            % ui32RequestedDegree: Requested degree to check against ui32MaxDegree; 0 means full file degree.
            % options.strScenarioSpec: Optional scenario specification from CScenarioRegistry.
            % options.bRequireScenarioMatch: Require charScenarioName and length units to match strScenarioSpec.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % CScenarioRegistry, EnumLengthUnits
            % -------------------------------------------------------------------------------------------------------------

            if ui32RequestedDegree == uint32(0)
                ui32RequestedDegree = self.ui32MaxDegree;
            end

            if self.ui32SchemaVersion ~= uint32(1) || ...
                    ~strcmp(char(self.charSavedDataConvention), SSphericalHarmonicsGravityData.charSavedDataConvention_) || ...
                    ~strcmp(char(self.charFormat), 'SSphericalHarmonicsGravityData') || ...
                    ~strcmp(char(self.charCoefficientStorage), 'cslm_cols') || ...
                    ~strcmp(char(self.charNormalization), 'unnormalized')
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'Invalid spherical-harmonics gravity data schema.');
            end

            if ~isstruct(self.strDocumentationHeader) || ...
                    ~isfield(self.strDocumentationHeader, 'charSavedDataConvention') || ...
                    ~strcmp(char(self.strDocumentationHeader.charSavedDataConvention), SSphericalHarmonicsGravityData.charSavedDataConvention_)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'Missing or invalid saved-data documentation header.');
            end

            if ~ismember(string(self.charLengthUnits), ["m", "km"])
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'charLengthUnits must be either "m" or "km".');
            end
            self.enumLengthUnits = EnumLengthUnits.fromAny(self.charLengthUnits);

            if ~isempty(strtrim(self.charScenarioName))
                [self.enumScenarioName, self.charScenarioName] = ...
                    SSphericalHarmonicsGravityData.NormalizeScenario_(self.charScenarioName);
            end

            if self.ui32MaxDegree < uint32(2)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'ui32MaxDegree must be at least 2.');
            end

            ui32ExpectedRows = SSphericalHarmonicsGravityData.CountCoeffRows_(self.ui32MaxDegree);
            if size(self.dCSlmCoeffCols, 1) ~= double(ui32ExpectedRows) || size(self.dCSlmCoeffCols, 2) ~= 2
                error('SSphericalHarmonicsGravityData:InvalidCoefficientRows', ...
                    'Degree %u requires %u coefficient rows, but file provides %u.', ...
                    self.ui32MaxDegree, ui32ExpectedRows, size(self.dCSlmCoeffCols, 1));
            end

            if ~isreal(self.dCSlmCoeffCols) || any(~isfinite(self.dCSlmCoeffCols), 'all')
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'dCSlmCoeffCols must contain finite real values.');
            end

            if ui32RequestedDegree < uint32(2) || ui32RequestedDegree > self.ui32MaxDegree
                error('SSphericalHarmonicsGravityData:RequestedDegreeUnavailable', ...
                    'Requested degree %u is unavailable. File max degree is %u.', ...
                    ui32RequestedDegree, self.ui32MaxDegree);
            end

            if ~isfinite(self.dGravParam) || ~(self.dGravParam > 0.0)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'dGravParam must be positive and finite.');
            end

            if ~isfinite(self.dBodyRadiusRef) || ~(self.dBodyRadiusRef > 0.0)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'dBodyRadiusRef must be positive and finite.');
            end

            if isfinite(self.dDensity) && ~(self.dDensity > 0.0)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'dDensity must be positive when specified.');
            end

            dExpectedGravConst = SSphericalHarmonicsGravityData.ResolveGravConstForLengthUnit_(self.charLengthUnits);
            dRelGravConstErr = abs(self.dGravConst - dExpectedGravConst) / dExpectedGravConst;
            if ~isfinite(self.dGravConst) || dRelGravConstErr > 1.0e-12
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'dGravConst must match the physical constant scaled to charLengthUnits.');
            end

            if options.bRequireScenarioMatch
                if ~isfield(options.strScenarioSpec, 'enumScenarioName')
                    error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                        'Scenario validation requested without a scenario spec.');
                end

                if isempty(strtrim(self.charScenarioName))
                    error('SSphericalHarmonicsGravityData:ScenarioMismatch', ...
                        'File-backed SH data must declare charScenarioName.');
                end

                try
                    [enumFileScenario, charFileCanonicalName] = CScenarioRegistry.ResolveScenario(self.enumScenarioName);
                catch ME
                    error('SSphericalHarmonicsGravityData:ScenarioMismatch', ...
                        'Unsupported file-backed SH scenario "%s": %s', self.charScenarioName, ME.message);
                end

                if enumFileScenario ~= options.strScenarioSpec.enumScenarioName
                    error('SSphericalHarmonicsGravityData:ScenarioMismatch', ...
                        'File-backed SH data for %s cannot be used for scenario %s.', ...
                        string(charFileCanonicalName), string(options.strScenarioSpec.charCanonicalName));
                end

                if isfield(options.strScenarioSpec, 'charLengthUnits') && ...
                        ~strcmpi(char(self.charLengthUnits), char(options.strScenarioSpec.charLengthUnits))
                    error('SSphericalHarmonicsGravityData:LengthUnitMismatch', ...
                        'File-backed SH data use %s, but scenario data use %s.', ...
                        self.charLengthUnits, char(options.strScenarioSpec.charLengthUnits));
                end
            end
        end

        function strSaved = toSavedStruct(self)
            arguments
                self (1,1) SSphericalHarmonicsGravityData
            end
            %% SIGNATURE
            % strSaved = toSavedStruct(self)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Convert the object to the plain scalar struct used as the repo-native saved-data payload.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self: Valid SSphericalHarmonicsGravityData object.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % strSaved: Scalar struct matching SimulationGears.SSphericalHarmonicsGravityData.v01.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SSphericalHarmonicsGravityData.validate
            % -------------------------------------------------------------------------------------------------------------

            self.validate();

            strSaved = struct( ...
                'strDocumentationHeader', self.strDocumentationHeader, ...
                'ui32SchemaVersion', self.ui32SchemaVersion, ...
                'charSavedDataConvention', self.charSavedDataConvention, ...
                'charFormat', self.charFormat, ...
                'charCoefficientStorage', self.charCoefficientStorage, ...
                'charNormalization', self.charNormalization, ...
                'charScenarioName', self.charScenarioName, ...
                'charLengthUnits', self.charLengthUnits, ...
                'ui32MaxDegree', self.ui32MaxDegree, ...
                'dCSlmCoeffCols', self.dCSlmCoeffCols, ...
                'dGravParam', self.dGravParam, ...
                'dBodyRadiusRef', self.dBodyRadiusRef, ...
                'dDensity', self.dDensity, ...
                'dGravConst', self.dGravConst, ...
                'charSource', self.charSource, ...
                'charSourceUrl', self.charSourceUrl, ...
                'charFieldKind', self.charFieldKind);
        end

        function saveToFile(self, charFilePath)
            arguments
                self (1,1) SSphericalHarmonicsGravityData
                charFilePath (1,:) {mustBeA(charFilePath, ["string", "char"])}
            end
            %% SIGNATURE
            % saveToFile(self, charFilePath)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Serialize the saved-data payload to MAT or JSON. MAT files store a single variable named
            % strSphericalHarmonicsGravityData.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self: Valid SSphericalHarmonicsGravityData object.
            % charFilePath: Output MAT or JSON file path; missing extension defaults to MAT.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % [-]
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SSphericalHarmonicsGravityData.toSavedStruct
            % -------------------------------------------------------------------------------------------------------------

            charFilePath = string(charFilePath);
            [charFolder, ~, charExt] = fileparts(charFilePath);
            if strlength(charFolder) > 0 && ~isfolder(charFolder)
                mkdir(charFolder);
            end
            if strlength(charExt) == 0
                charExt = ".mat";
                charFilePath = charFilePath + charExt;
            end

            strSphericalHarmonicsGravityData = self.toSavedStruct(); %#ok<NASGU>
            switch lower(charExt)
                case ".mat"
                    save(charFilePath, 'strSphericalHarmonicsGravityData');
                case ".json"
                    charJson = jsonencode(strSphericalHarmonicsGravityData, "PrettyPrint", true);
                    fileID = fopen(char(charFilePath), 'w');
                    if fileID < 0
                        error('SSphericalHarmonicsGravityData:FileWriteError', ...
                            'Unable to open %s for writing.', char(charFilePath));
                    end
                    objCleanup = onCleanup(@() fclose(fileID));
                    fprintf(fileID, '%s', charJson);
                    clear objCleanup
                otherwise
                    error('SSphericalHarmonicsGravityData:UnsupportedFileFormat', ...
                        'Unsupported SH data file extension "%s".', charExt);
            end
        end

        function strGravityData = toGravityDataStruct(self, ui32RequestedDegree)
            arguments
                self (1,1) SSphericalHarmonicsGravityData
                ui32RequestedDegree (1,1) uint32 = uint32(0)
            end
            %% SIGNATURE
            % strGravityData = toGravityDataStruct(self, ui32RequestedDegree)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Build the gravity-data struct consumed by dynamics and registry callers, truncating the coefficient
            % table to the requested degree.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self: Valid SSphericalHarmonicsGravityData object.
            % ui32RequestedDegree: Requested output degree; 0 means full file degree.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % strGravityData: Runtime gravity-data struct with coefficients, GM, radius, density, and metadata.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SSphericalHarmonicsGravityData.validate, SSphericalHarmonicsGravityData.toMetadataStruct
            % -------------------------------------------------------------------------------------------------------------

            if ui32RequestedDegree == uint32(0)
                ui32RequestedDegree = self.ui32MaxDegree;
            end

            self.validate(ui32RequestedDegree);

            ui32RequiredRows = SSphericalHarmonicsGravityData.CountCoeffRows_(ui32RequestedDegree);
            strMetadata = self.toMetadataStruct();
            strGravityData = struct( ...
                'dCSlmCoeffCols', self.dCSlmCoeffCols(1:double(ui32RequiredRows), :), ...
                'ui32MaxDegree', ui32RequestedDegree, ...
                'dGravParam', self.dGravParam, ...
                'dBodyRadiusRef', self.dBodyRadiusRef, ...
                'dDensity', self.dDensity, ...
                'dGravConst', self.dGravConst, ...
                'strFitStats', strMetadata, ...
                'bRegistryBacked', false, ...
                'bFileBacked', true, ...
                'strRegistryMetadata', strMetadata);
        end

        function strMetadata = toMetadataStruct(self)
            arguments
                self (1,1) SSphericalHarmonicsGravityData
            end
            %% SIGNATURE
            % strMetadata = toMetadataStruct(self)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Return provenance, storage, unit, and availability metadata without exposing the coefficient table.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % self: SSphericalHarmonicsGravityData object.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % strMetadata: Scalar metadata struct for load reporting and registry-compatible paths.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % [-]
            % -------------------------------------------------------------------------------------------------------------

            strMetadata = struct( ...
                'bFileBacked', true, ...
                'bRegistryBacked', false, ...
                'bHasOnlineSource', false, ...
                'bHasHardcodedCoefficients', false, ...
                'bUseByDefault', false, ...
                'ui32SourceMaxDegree', self.ui32MaxDegree, ...
                'ui32HardcodedMaxDegree', uint32(0), ...
                'ui32FileMaxDegree', self.ui32MaxDegree, ...
                'dCSlmCoeffCols', zeros(0, 2), ...
                'dGravParam', self.dGravParam, ...
                'dBodyRadiusRef', self.dBodyRadiusRef, ...
                'dDensity', self.dDensity, ...
                'dGravConst', self.dGravConst, ...
                'charLengthUnits', string(self.charLengthUnits), ...
                'charNormalization', string(self.charNormalization), ...
                'charStoredFormat', "Repo-native unnormalized [Clm, Slm] column pairs", ...
                'charCoefficientStorage', string(self.charCoefficientStorage), ...
                'charFieldKind', string(self.charFieldKind), ...
                'charSource', string(self.charSource), ...
                'charSourceUrl', string(self.charSourceUrl), ...
                'charScenarioName', string(self.charScenarioName), ...
                'strDocumentationHeader', self.strDocumentationHeader);
        end
    end

    methods (Static, Access = public)
        function objData = fromFile(charFilePath)
            arguments
                charFilePath (1,:) {mustBeA(charFilePath, ["string", "char"])}
            end
            %% SIGNATURE
            % objData = SSphericalHarmonicsGravityData.fromFile(charFilePath)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Load a MAT or JSON spherical-harmonics gravity data file and validate it against the saved-data schema.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % charFilePath: Input MAT or JSON file path.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objData: Validated SSphericalHarmonicsGravityData object.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SSphericalHarmonicsGravityData.fromSavedStruct
            % -------------------------------------------------------------------------------------------------------------

            charFilePath = string(charFilePath);
            mustBeFile(charFilePath);
            [~, ~, charExt] = fileparts(charFilePath);

            switch lower(charExt)
                case ".mat"
                    strLoaded = load(charFilePath);
                    if ~isfield(strLoaded, SSphericalHarmonicsGravityData.charSavedStructVariable_)
                        error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                            'MAT file must contain variable %s.', SSphericalHarmonicsGravityData.charSavedStructVariable_);
                    end
                    strInput = strLoaded.(SSphericalHarmonicsGravityData.charSavedStructVariable_);
                case ".json"
                    strInput = jsondecode(fileread(char(charFilePath)));
                otherwise
                    error('SSphericalHarmonicsGravityData:UnsupportedFileFormat', ...
                        'Unsupported SH data file extension "%s".', charExt);
            end

            objData = SSphericalHarmonicsGravityData.fromSavedStruct(strInput);
        end

        function objData = fromSavedStruct(strInput)
            arguments
                strInput
            end
            %% SIGNATURE
            % objData = SSphericalHarmonicsGravityData.fromSavedStruct(strInput)
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Restore a spherical-harmonics gravity data object from the plain saved struct or a wrapper struct,
            % rejecting missing, unknown, or inconsistent fields before runtime use.
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % strInput: Saved scalar struct, wrapper struct, or SSphericalHarmonicsGravityData object.
            % -------------------------------------------------------------------------------------------------------------
            %% OUTPUT
            % objData: Validated SSphericalHarmonicsGravityData object.
            % -------------------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % SSphericalHarmonicsGravityData.validate, EnumLengthUnits, CScenarioRegistry
            % -------------------------------------------------------------------------------------------------------------

            if isa(strInput, 'SSphericalHarmonicsGravityData')
                objData = strInput;
                objData.validate();
                return
            end

            if ~isstruct(strInput) || ~isscalar(strInput)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'Saved SH data must be a scalar struct.');
            end

            if isfield(strInput, SSphericalHarmonicsGravityData.charSavedStructVariable_)
                strInput = strInput.(SSphericalHarmonicsGravityData.charSavedStructVariable_);
            end

            SSphericalHarmonicsGravityData.RequireFields_(strInput, { ...
                'strDocumentationHeader', ...
                'ui32SchemaVersion', ...
                'charSavedDataConvention', ...
                'charFormat', ...
                'charCoefficientStorage', ...
                'charNormalization', ...
                'charScenarioName', ...
                'charLengthUnits', ...
                'ui32MaxDegree', ...
                'dCSlmCoeffCols', ...
                'dGravParam', ...
                'dBodyRadiusRef', ...
                'dDensity', ...
                'dGravConst', ...
                'charSource', ...
                'charSourceUrl', ...
                'charFieldKind'});

            cellKnownFields = { ...
                'strDocumentationHeader', ...
                'ui32SchemaVersion', ...
                'charSavedDataConvention', ...
                'charFormat', ...
                'charCoefficientStorage', ...
                'charNormalization', ...
                'charScenarioName', ...
                'charLengthUnits', ...
                'ui32MaxDegree', ...
                'dCSlmCoeffCols', ...
                'dGravParam', ...
                'dBodyRadiusRef', ...
                'dDensity', ...
                'dGravConst', ...
                'charSource', ...
                'charSourceUrl', ...
                'charFieldKind'};
            cellUnknownFields = setdiff(fieldnames(strInput), cellKnownFields);
            if ~isempty(cellUnknownFields)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'Unknown fields in saved SH data: %s.', strjoin(cellUnknownFields', ', '));
            end

            objData = SSphericalHarmonicsGravityData();
            objData.strDocumentationHeader = strInput.strDocumentationHeader;
            objData.ui32SchemaVersion = uint32(strInput.ui32SchemaVersion);
            objData.charSavedDataConvention = char(strInput.charSavedDataConvention);
            objData.charFormat = char(strInput.charFormat);
            objData.charCoefficientStorage = char(strInput.charCoefficientStorage);
            objData.charNormalization = char(strInput.charNormalization);
            [objData.enumScenarioName, objData.charScenarioName] = ...
                SSphericalHarmonicsGravityData.NormalizeScenario_(strInput.charScenarioName);
            objData.enumLengthUnits = EnumLengthUnits.fromAny(strInput.charLengthUnits);
            objData.charLengthUnits = char(EnumLengthUnits.toString(objData.enumLengthUnits));
            objData.ui32MaxDegree = uint32(strInput.ui32MaxDegree);
            objData.dCSlmCoeffCols = double(strInput.dCSlmCoeffCols);
            objData.dGravParam = double(strInput.dGravParam);
            objData.dBodyRadiusRef = double(strInput.dBodyRadiusRef);
            objData.dDensity = SSphericalHarmonicsGravityData.RestoreNullableScalar_(strInput.dDensity);
            objData.dGravConst = SSphericalHarmonicsGravityData.RestoreNullableScalar_(strInput.dGravConst);
            objData.charSource = char(strInput.charSource);
            objData.charSourceUrl = char(strInput.charSourceUrl);
            objData.charFieldKind = char(strInput.charFieldKind);
            objData.bDefaultConstructed = false;

            objData.validate();
        end
    end

    methods (Static, Access = private)
        function strHeader = BuildDocumentationHeader_()
            strHeader = struct( ...
                'charSavedDataConvention', SSphericalHarmonicsGravityData.charSavedDataConvention_, ...
                'charSavedStructVariable', SSphericalHarmonicsGravityData.charSavedStructVariable_, ...
                'charCoefficientStorage', 'cslm_cols', ...
                'charNormalization', 'unnormalized', ...
                'charRowConvention', ['row 1 is degree-1 compatibility row; rows for degree >= 2 follow ' ...
                    'in increasing degree and order m=0..l'], ...
                'charUnitsConvention', ['dGravParam in charLengthUnits^3/s^2; dBodyRadiusRef in charLengthUnits; ' ...
                    'dGravConst scaled from SI by charLengthUnits']);
        end

        function ui32Count = CountCoeffRows_(ui32MaxDegree)
            arguments
                ui32MaxDegree (1,1) uint32
            end

            ui32Count = uint32((double(ui32MaxDegree) + 1.0) * (double(ui32MaxDegree) + 2.0) / 2.0 - 2.0);
        end

        function dGravConst = ResolveGravConstForLengthUnit_(charLengthUnits)
            arguments
                charLengthUnits {mustBeA(charLengthUnits, ["string", "char", "EnumLengthUnits"])}
            end

            switch EnumLengthUnits.toString(charLengthUnits)
                case "m"
                    dGravConst = 6.67430e-11;
                case "km"
                    dGravConst = 6.67430e-20;
                otherwise
                    error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                        'Unsupported length unit "%s".', char(charLengthUnits));
            end
        end

        function [enumScenarioName, charScenarioName] = NormalizeScenario_(varScenarioName)
            arguments
                varScenarioName (1,:) {mustBeA(varScenarioName, ["string", "char", "EnumScenarioName"])}
            end

            if ~isa(varScenarioName, "EnumScenarioName") && strlength(string(varScenarioName)) == 0
                enumScenarioName = EnumScenarioName.NotDefined;
                charScenarioName = '';
                return
            end

            [enumScenarioName, charCanonicalName] = CScenarioRegistry.ResolveScenario(varScenarioName);
            charScenarioName = char(charCanonicalName);
        end

        function RequireFields_(strInput, cellRequiredFields)
            arguments
                strInput (1,1) struct
                cellRequiredFields (1,:) cell
            end

            cellMissingFields = cellRequiredFields(~isfield(strInput, cellRequiredFields));
            if ~isempty(cellMissingFields)
                error('SSphericalHarmonicsGravityData:InvalidSchema', ...
                    'Missing fields in saved SH data: %s.', strjoin(cellMissingFields, ', '));
            end
        end

        function dValue = RestoreNullableScalar_(varValue)
            if isempty(varValue)
                dValue = NaN;
            else
                dValue = double(varValue);
            end
        end
    end
end
