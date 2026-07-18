classdef EnumLengthUnits
    %% DESCRIPTION
    % Shared length-unit enumeration for SimulationGears public APIs.
    % -------------------------------------------------------------------------------------------------------------
    %% ENUMERATIONS
    % km: kilometers.
    % m: meters.
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % fromAny: Convert char/string/EnumLengthUnits inputs to EnumLengthUnits.
    % toString: Convert any supported length-unit input to its canonical string.
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    enumeration
        km
        m
    end

    methods (Static)
        function enumLengthUnits = fromAny(varLengthUnits)
            %% DESCRIPTION
            % Convert supported length-unit inputs to EnumLengthUnits.
            % -------------------------------------------------------------------------------------------------------------
            arguments
                varLengthUnits {mustBeA(varLengthUnits, ["EnumLengthUnits", "string", "char"])}
            end

            if isa(varLengthUnits, "EnumLengthUnits")
                enumLengthUnits = varLengthUnits;
                return
            end

            switch string(varLengthUnits)
                case "m"
                    enumLengthUnits = EnumLengthUnits.m;
                case "km"
                    enumLengthUnits = EnumLengthUnits.km;
                otherwise
                    error("EnumLengthUnits:UnsupportedUnit", ...
                        "Unsupported length unit ""%s"". Expected ""m"" or ""km"".", ...
                        string(varLengthUnits));
            end
        end

        function charLengthUnits = toString(varLengthUnits)
            %% DESCRIPTION
            % Convert supported length-unit inputs to canonical string values.
            % -------------------------------------------------------------------------------------------------------------
            arguments
                varLengthUnits {mustBeA(varLengthUnits, ["EnumLengthUnits", "string", "char"])}
            end

            charLengthUnits = string(EnumLengthUnits.fromAny(varLengthUnits));
        end
    end
end
