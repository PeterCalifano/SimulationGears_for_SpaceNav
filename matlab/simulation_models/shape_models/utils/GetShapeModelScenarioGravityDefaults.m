function strGravityDefaults = GetShapeModelScenarioGravityDefaults(enumTargetName, charLengthUnits)
arguments
    enumTargetName  (1,:) {mustBeA(enumTargetName, ["string", "char", "EnumScenarioName"])}
    charLengthUnits (1,:) string {mustBeA(charLengthUnits, ["string", "char"]), ...
        mustBeMember(charLengthUnits, ["m", "km"])} = "m"
end
%% DESCRIPTION
% Compatibility wrapper around CScenarioRegistry.GetGravityDefaults().
% Returns known-scenario gravity defaults in the length units used by the
% active shape model. Unknown/custom scenarios return NaN values and
% bHasDefaults=false so callers can require explicit physical inputs.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strGravityDefaults.bHasDefaults [logical] true when a known default was found.
% strGravityDefaults.dGravParam   [LU^3/s^2] gravitational parameter.
% strGravityDefaults.dDensity     [mass/LU^3] NaN unless explicitly registered.
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
