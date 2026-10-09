classdef EnumSurfaceMapMapping < uint8
%% DESCRIPTION
% Mapping used to associate a registered surface image with target geometry.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6  First typed surface-map mapping contract.
% -------------------------------------------------------------------------------------------------------------
%% ENUMERATIONS
% NONE, MESH_UV, BODY_FIXED_EQUIRECTANGULAR.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

enumeration
    NONE (0)
    MESH_UV (1)
    BODY_FIXED_EQUIRECTANGULAR (2)
end

methods (Static)
    function enumMapping = FromConfigValue(charMapping)
        %% SIGNATURE
        % enumMapping = EnumSurfaceMapMapping.FromConfigValue(charMapping)
        % -----------------------------------------------------------------------------------------------------
        %% DESCRIPTION
        % Convert a manifest string into a surface-map mapping value.
        % -----------------------------------------------------------------------------------------------------
        %% INPUT
        % charMapping   (1,:) char or string; mapping name.
        % -----------------------------------------------------------------------------------------------------
        %% OUTPUT
        % enumMapping   (1,1) EnumSurfaceMapMapping; normalized value.
        % -----------------------------------------------------------------------------------------------------
        %% CHANGELOG
        % 21-09-2026  Pietro Califano, Codex gpt-5.6  First implementation.
        % -----------------------------------------------------------------------------------------------------
        %% DEPENDENCIES
        % None.
        % -----------------------------------------------------------------------------------------------------
        arguments
            charMapping (1,:) {mustBeText}
        end

        switch upper(string(charMapping))
            case {"", "NONE"}
                enumMapping = EnumSurfaceMapMapping.NONE;
            case "MESH_UV"
                enumMapping = EnumSurfaceMapMapping.MESH_UV;
            case "BODY_FIXED_EQUIRECTANGULAR"
                enumMapping = EnumSurfaceMapMapping.BODY_FIXED_EQUIRECTANGULAR;
            otherwise
                error("EnumSurfaceMapMapping:UnsupportedMapping", ...
                    "Unsupported surface-map mapping '%s'.", string(charMapping));
        end
    end
end
end
