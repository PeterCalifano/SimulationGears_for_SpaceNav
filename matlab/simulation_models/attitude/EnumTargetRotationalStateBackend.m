classdef EnumTargetRotationalStateBackend < uint8
    %% SIGNATURE
    % enumBackend = EnumTargetRotationalStateBackend.<Value>
    % -------------------------------------------------------------------------------------------------------------
    %% DESCRIPTION
    % Select the numerical backend used to integrate a generated target rotational-state history. MATLAB executes
    % the reference kernel without generated-artifact access; MEX explicitly requests compatible artifact
    % build/reuse.
    % -------------------------------------------------------------------------------------------------------------
    %% INPUT
    % None.
    % -------------------------------------------------------------------------------------------------------------
    %% OUTPUT
    % enumBackend    (1,1) EnumTargetRotationalStateBackend
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 24-08-2026  Pietro Califano, Codex     First implementation.
    % 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % None.
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        MATLAB (0)
        MEX    (1)
    end

    methods (Static)
        function enumBackend = FromConfigValue(varBackend)
            %% SIGNATURE
            % enumBackend = EnumTargetRotationalStateBackend.FromConfigValue(varBackend)
            % -------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Convert a typed value or case-insensitive MATLAB/MEX configuration token into the backend enum.
            % -------------------------------------------------------------------------------------------------
            %% INPUT
            % varBackend    EnumTargetRotationalStateBackend or scalar text token
            % -------------------------------------------------------------------------------------------------
            %% OUTPUT
            % enumBackend   (1,1) EnumTargetRotationalStateBackend
            % -------------------------------------------------------------------------------------------------
            %% CHANGELOG
            % 24-08-2026  Pietro Califano, Codex     First implementation.
            % -------------------------------------------------------------------------------------------------
            %% DEPENDENCIES
            % None.
            % -------------------------------------------------------------------------------------------------

            arguments (Input)
                varBackend {mustBeA(varBackend, ["EnumTargetRotationalStateBackend", "char", "string"])}
            end
            arguments (Output)
                enumBackend (1,1) EnumTargetRotationalStateBackend
            end

            % Accept a scalar enum value as is
            if isa(varBackend, 'EnumTargetRotationalStateBackend')
                if ~isscalar(varBackend)
                    error('EnumTargetRotationalStateBackend:NonScalarBackend', ...
                        'Target rotational-state backend must be scalar.');
                end
                enumBackend = varBackend;
                return
            end

            % Map a text token to the enum, ignoring case and surrounding blanks
            mustBeTextScalar(varBackend);
            switch upper(char(strip(string(varBackend))))
                case 'MATLAB'
                    enumBackend = EnumTargetRotationalStateBackend.MATLAB;
                case 'MEX'
                    enumBackend = EnumTargetRotationalStateBackend.MEX;
                otherwise
                    error('EnumTargetRotationalStateBackend:InvalidConfigValue', ...
                        'Target rotational-state backend must be MATLAB or MEX.');
            end
        end
    end
end
