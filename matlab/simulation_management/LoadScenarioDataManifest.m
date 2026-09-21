function strManifest = LoadScenarioDataManifest(enumOrName, options)
%% DESCRIPTION
% Load and validate the tracked manifest for a SimulationGears scenario.
% -------------------------------------------------------------------------------------------------------------
%% SIGNATURE
% strManifest = LoadScenarioDataManifest(enumOrName, options)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumOrName                 (1,:) string, char, or EnumScenarioName identifying the scenario.
% options.charDataRootPath   (1,:) string = ""; override for the SimulationGears data root.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strManifest                (1,1) struct decoded from the scenario manifest JSON and validated.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6  Keep manifest and payload root resolution separate.
% 01-07-2026  Pietro Califano     Add SimulationGears scenario manifest loader.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CScenarioRegistry, ResolveSimGearsDataRoot, NormalizeManifestStructArray,
% ValidateScenarioDataManifest, jsondecode.
% -------------------------------------------------------------------------------------------------------------

arguments
    enumOrName (1,:) {mustBeA(enumOrName, ["string", "char", "EnumScenarioName"])}
    options.charDataRootPath (1,:) string = ""
end

charDataRootPath = ResolveSimGearsDataRoot(charDataRootPath=options.charDataRootPath);
[~, charCanonicalName] = CScenarioRegistry.ResolveScenario(enumOrName);
strScenarioSpec = CScenarioRegistry.GetScenarioSpec(charCanonicalName);

if strlength(string(strScenarioSpec.charDataManifestRelativePath)) == 0
    error("LoadScenarioDataManifest:ManifestUnavailable", ...
        "Scenario %s is not backed by a SimulationGears data manifest.", charCanonicalName);
end

charManifestPath = fullfile(charDataRootPath, strScenarioSpec.charDataManifestRelativePath);
if ~isfile(charManifestPath)
    error("LoadScenarioDataManifest:MissingManifest", ...
        "Scenario manifest for %s not found at %s.", charCanonicalName, string(charManifestPath));
end

strManifest = jsondecode(fileread(char(charManifestPath)));
strManifest.assets = NormalizeManifestStructArray(strManifest.assets);
if isfield(strManifest, "appearance_profiles")
    strManifest.appearance_profiles = NormalizeManifestStructArray(strManifest.appearance_profiles);
end
ValidateScenarioDataManifest(strManifest, charCanonicalName, charDataRootPath=charDataRootPath);
end
