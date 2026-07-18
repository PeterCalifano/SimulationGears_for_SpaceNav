function strGravityDefaults = GetShapeModelScenarioGravityDefaults(enumTargetName, charLengthUnits)
arguments
    enumTargetName  (1,:) {mustBeA(enumTargetName, ["string", "char", "EnumScenarioName"])}
    charLengthUnits {mustBeA(charLengthUnits, ["string", "char", "EnumLengthUnits"])} = "m"
end
%% SIGNATURE
% strGravityDefaults = GetShapeModelScenarioGravityDefaults(enumTargetName, charLengthUnits)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compatibility wrapper around CScenarioRegistry.GetGravityDefaults().
% Returns known-scenario gravity defaults in the length units used by the
% active shape model. Unknown/custom scenarios return NaN values and
% bHasDefaults=false so callers can require explicit physical inputs.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumTargetName:   [1] Known scenario name, alias, or EnumScenarioName.
% charLengthUnits:  [1] Requested length units ('m' or 'km').
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strGravityDefaults.bHasDefaults [logical] true when a known default was found.
% strGravityDefaults.dGravParam   [LU^3/s^2] gravitational parameter.
% strGravityDefaults.dDensity     [mass/LU^3] NaN unless explicitly registered.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-07-2026    Pietro Califano     Route scenario gravity defaults through CScenarioRegistry with enum units.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CScenarioRegistry.GetGravityDefaults()
% EnumLengthUnits
% -------------------------------------------------------------------------------------------------------------

try
    strGravityDefaults = CScenarioRegistry.GetGravityDefaults(enumTargetName, charLengthUnits);

catch objException
    if strcmp(objException.identifier, 'CScenarioRegistry:UnsupportedScenario')
        strGravityDefaults = struct( ...
            'bHasDefaults', false, ...
            'dGravParam', NaN, ...
            'dDensity', NaN);
        return
    end
    rethrow(objException)
end

end
