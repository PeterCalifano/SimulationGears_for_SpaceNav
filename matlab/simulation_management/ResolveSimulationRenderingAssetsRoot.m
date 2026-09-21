function charAssetRootPath = ResolveSimulationRenderingAssetsRoot(options)
%% SIGNATURE
% charAssetRootPath = ResolveSimulationRenderingAssetsRoot(options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve the shared external simulation-rendering asset root. The root owns
% large runtime payloads referenced by tracked SimulationGears manifests.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% options.charAssetRootPath (1,:) string = ""; explicit root override.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% charAssetRootPath         (1,:) string; existing external asset root.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6  Separate external assets from tracked registry data.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% getenv, isfolder.
% -------------------------------------------------------------------------------------------------------------

arguments
    options.charAssetRootPath (1,:) string = ""
end

if strlength(options.charAssetRootPath) > 0
    charAssetRootPath = string(options.charAssetRootPath);
else
    charAssetRootPath = string(getenv("RENDERING_DATA"));
end

if strlength(charAssetRootPath) == 0 || ~isfolder(charAssetRootPath)
    error("ResolveSimulationRenderingAssetsRoot:MissingAssetRoot", ...
        ["Simulation rendering asset root not found. Pass charAssetRootPath " ...
         "or set RENDERING_DATA to the shared simulation_rendering_assets directory."]);
end
end
