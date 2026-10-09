function charAssetPath = ResolveScenarioAssetPath(charRelativePath, options)
%% SIGNATURE
% charAssetPath = ResolveScenarioAssetPath(charRelativePath, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve a manifest payload path against the shared external asset root.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charRelativePath             (1,:) char or string; path stored in a manifest.
% options.charAssetRootPath    (1,:) string = ""; explicit external-root override.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% charAssetPath                (1,:) string; absolute payload path.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6  First external payload resolver.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveSimulationRenderingAssetsRoot.
% -------------------------------------------------------------------------------------------------------------

arguments
    charRelativePath (1,:) {mustBeText}
    options.charAssetRootPath (1,:) string = ""
end

charRelativePath = string(charRelativePath);
if startsWith(charRelativePath, filesep)
    charAssetPath = charRelativePath;
    return
end

charAssetRootPath = ResolveSimulationRenderingAssetsRoot( ...
    charAssetRootPath=options.charAssetRootPath);
charAssetPath = string(fullfile(charAssetRootPath, charRelativePath));
end
