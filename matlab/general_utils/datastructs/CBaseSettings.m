classdef CBaseSettings < CBaseDatastruct
    %% DESCRIPTION
    % Base class for settings objects. It extends CBaseDatastruct with small, non-compute-intensive utilities shared
    % by concrete settings classes and configuration builders.
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 09-02-2025    Pietro Califano             Defined as empty class for future developments in nav-frontend.
    % 01-07-2026    Pietro Califano, Codex 5.5  Move shared settings helpers to SimulationGears ownership.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % checkDefaultConstruction: mark settings as non-default when name-value inputs differ from defaults.
    % CopyStructToKnownFields: copy a struct into a known override schema and reject unknown fields.
    % ApplyStructToSettingsObject: assign a struct to a settings object and reject unknown properties.
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % Inherited from CBaseDatastruct.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % CBaseDatastruct
    % -------------------------------------------------------------------------------------------------------------

    methods (Access = public)
        function self = CBaseSettings()
            arguments
            end
        end

        function checkDefaultConstruction(self, strKwargs)
            arguments
                self
                strKwargs (1,1) struct
            end

            cellFields = fieldnames(strKwargs);
            for ui32FieldIdx = 1:numel(cellFields)
                charFieldName = cellFields{ui32FieldIdx};

                if ~isequal(strKwargs.(charFieldName), self.(charFieldName))
                    self.bDefaultConstructed = false;
                    return
                end
            end
        end
    end

    methods (Access = public, Static)
        function strOverrides = CopyStructToKnownFields(strOverrides, strSettings, charErrorId)
            % Copy fields from an input settings struct into a known override schema.
            % This is intentionally strict for YAML/config ingestion: unknown fields are errors rather than ignored
            % overrides, so stale config files fail close to the loader.
            arguments
                strOverrides (1,1) struct
                strSettings  (1,1) struct
                charErrorId  (1,:) char = 'CBaseSettings:UnknownSettingsField'
            end

            cellFieldNames = fieldnames(strSettings);
            for ui32FieldIdx = 1:numel(cellFieldNames)
                charFieldName = cellFieldNames{ui32FieldIdx};
                if ~isfield(strOverrides, charFieldName)
                    error(charErrorId, 'Unknown settings field "%s".', charFieldName);
                end
                strOverrides.(charFieldName) = strSettings.(charFieldName);
            end
        end

        function objSettings = ApplyStructToSettingsObject(objSettings, strSettings, charErrorId)
            % Apply fields from an input settings struct directly to a settings object.
            % This is the object-side counterpart of CopyStructToKnownFields. It is intended for configuration
            % builders, not for hot loops.
            arguments
                objSettings (1,1)
                strSettings (1,1) struct
                charErrorId (1,:) char = 'CBaseSettings:UnknownSettingsField'
            end

            cellFieldNames = fieldnames(strSettings);
            for ui32FieldIdx = 1:numel(cellFieldNames)
                charFieldName = cellFieldNames{ui32FieldIdx};
                if ~isprop(objSettings, charFieldName)
                    error(charErrorId, 'Unknown settings field "%s".', charFieldName);
                end
                objSettings.(charFieldName) = strSettings.(charFieldName);
            end

            if ~isempty(cellFieldNames) && isprop(objSettings, 'bDefaultConstructed')
                objSettings.bDefaultConstructed = false;
            end
        end
    end
end
