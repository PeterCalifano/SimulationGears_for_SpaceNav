classdef EnumTargetRotationalStateSource < uint8
    %% SIGNATURE
    % enumSource = EnumTargetRotationalStateSource.<Value>
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Select the authoritative source of a target rotational-state history.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % enumSource    (1,1) EnumTargetRotationalStateSource
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 22-08-2026  Pietro Califano, Codex     First implementation.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        SPICE     (1)
        DATASET   (2)
        GENERATED (3)
    end

    methods (Static)
        function enumSource = FromConfigValue(varSource)
            %% SIGNATURE
            % enumSource = EnumTargetRotationalStateSource.FromConfigValue(varSource)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Normalize a scalar configuration token or preserve an existing typed source.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % varSource    EnumTargetRotationalStateSource, char, or string scalar
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % enumSource   (1,1) EnumTargetRotationalStateSource
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                varSource {mustBeA(varSource, ["EnumTargetRotationalStateSource", "char", "string"])}
            end
            arguments (Output)
                enumSource (1,1) EnumTargetRotationalStateSource
            end

            % Accept a scalar enum value as is
            if isa(varSource, 'EnumTargetRotationalStateSource')
                if ~isscalar(varSource)
                    error('EnumTargetRotationalStateSource:NonScalarSource', ...
                        'Target rotational-state source must be scalar.');
                end
                enumSource = varSource;
                return
            end

            % Map a text token to the enum, ignoring case and surrounding blanks and reading '-' as '_'
            mustBeTextScalar(varSource);
            charSource = upper(strrep(char(strip(string(varSource))), '-', '_'));
            switch charSource
                case 'SPICE'
                    enumSource = EnumTargetRotationalStateSource.SPICE;
                case 'DATASET'
                    enumSource = EnumTargetRotationalStateSource.DATASET;
                case 'GENERATED'
                    enumSource = EnumTargetRotationalStateSource.GENERATED;
                otherwise
                    error('EnumTargetRotationalStateSource:UnsupportedSource', ...
                        'Unsupported target rotational-state source "%s".', charSource);
            end
        end

        function charSource = ToConfigValue(enumSource)
            %% SIGNATURE
            % charSource = EnumTargetRotationalStateSource.ToConfigValue(enumSource)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Convert a typed source to its canonical lowercase configuration token.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % enumSource    (1,1) EnumTargetRotationalStateSource
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % charSource    (1,:) char canonical configuration token
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 22-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                enumSource (1,1) EnumTargetRotationalStateSource
            end
            arguments (Output)
                charSource (1,:) char
            end

            switch enumSource
                case EnumTargetRotationalStateSource.SPICE
                    charSource = 'spice';
                case EnumTargetRotationalStateSource.DATASET
                    charSource = 'dataset';
                case EnumTargetRotationalStateSource.GENERATED
                    charSource = 'generated';
                otherwise
                    error('EnumTargetRotationalStateSource:UnsupportedEnumValue', ...
                        'Unsupported typed target rotational-state source.');
            end
        end
    end
end
