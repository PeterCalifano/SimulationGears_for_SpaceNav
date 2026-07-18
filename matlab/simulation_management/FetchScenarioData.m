function strFetchPlan = FetchScenarioData(enumOrName, options)
%% DESCRIPTION
% Fetch or plan scenario data assets described by SimulationGears manifests.
% -------------------------------------------------------------------------------------------------------------
%% SIGNATURE
% strFetchPlan = FetchScenarioData(enumOrName, options)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumOrName                   (1,:) string, char, or EnumScenarioName identifying the scenario.
% options.charDataRootPath     (1,:) string = ""; override for the SimulationGears data root.
% options.cellAssetIds         (1,:) cell = {}; optional manifest asset ids to fetch or plan.
% options.bDryRun              (1,1) logical = false; return the fetch plan without downloading.
% options.bRequireLocalAssets  (1,1) logical = false; validate required local assets before planning.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strFetchPlan                 (1,1) struct with scenario, data-root, dry-run, and per-asset plan fields.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-07-2026    Pietro Califano     Add MATLAB-side scenario asset planning and fetch helper.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CScenarioRegistry, ResolveSimGearsDataRoot, LoadScenarioDataManifest, ValidateScenarioDataManifest, websave.
% -------------------------------------------------------------------------------------------------------------

arguments
    enumOrName (1,:) {mustBeA(enumOrName, ["string", "char", "EnumScenarioName"])}
    options.charDataRootPath (1,:) string = ""
    options.cellAssetIds (1,:) cell = {}
    options.bDryRun (1,1) logical = false
    options.bRequireLocalAssets (1,1) logical = false
end

charDataRootPath = ResolveSimGearsDataRoot(charDataRootPath=options.charDataRootPath);
[~, charCanonicalName] = CScenarioRegistry.ResolveScenario(enumOrName);
strScenarioSpec = CScenarioRegistry.GetScenarioSpec(charCanonicalName);
charManifestPath = fullfile(charDataRootPath, strScenarioSpec.charDataManifestRelativePath);
strManifest = LoadScenarioDataManifest(charCanonicalName, charDataRootPath=charDataRootPath);
ValidateScenarioDataManifest(strManifest, charCanonicalName, ...
    charDataRootPath=charDataRootPath, ...
    bRequireLocalAssets=options.bRequireLocalAssets);

strAssets = strManifest.assets;
if ~isempty(options.cellAssetIds)
    cellRequestedIds = string(options.cellAssetIds);
    bKeepAsset = ismember(string({strAssets.asset_id}), cellRequestedIds);
    if any(~ismember(cellRequestedIds, string({strAssets.asset_id})))
        cellMissing = cellRequestedIds(~ismember(cellRequestedIds, string({strAssets.asset_id})));
        error("FetchScenarioData:UnknownAsset", ...
            "Manifest for %s does not define requested asset(s): %s.", ...
            charCanonicalName, strjoin(cellMissing, ", "));
    end
    strAssets = strAssets(bKeepAsset);
end

strFetchAssets = repmat(struct( ...
    "charAssetId", "", ...
    "charAssetType", "", ...
    "charLocalPath", "", ...
    "charDownloadUrl", "", ...
    "bExists", false, ...
    "bWouldDownload", false, ...
    "bDownloaded", false), 1, numel(strAssets));

for idxAsset = 1:numel(strAssets)
    charLocalPath = string(fullfile(charDataRootPath, string(strAssets(idxAsset).local_path)));
    bExists = isfile(charLocalPath) || isfolder(charLocalPath);
    bWouldDownload = ~bExists;

    strFetchAssets(idxAsset).charAssetId = string(strAssets(idxAsset).asset_id);
    strFetchAssets(idxAsset).charAssetType = string(strAssets(idxAsset).asset_type);
    strFetchAssets(idxAsset).charLocalPath = charLocalPath;
    strFetchAssets(idxAsset).charDownloadUrl = string(strAssets(idxAsset).download_url);
    strFetchAssets(idxAsset).bExists = bExists;
    strFetchAssets(idxAsset).bWouldDownload = bWouldDownload;

    if options.bDryRun || bExists
        continue
    end

    if strlength(string(strAssets(idxAsset).download_url)) == 0
        charFetchCommand = sprintf('python3 tools/data/fetch_scenario_assets.py --scenario %s --asset-id %s --verify-only', ...
            charCanonicalName, string(strAssets(idxAsset).asset_id));
        error("FetchScenarioData:MissingDownloadUrl", ...
            ['Asset %s for %s is missing and has no download_url in the manifest.\n' ...
             'Expected: %s\n' ...
             'Manifest: %s\n' ...
             'Place the asset manually or update download_url, then verify with:\n' ...
             '  %s'], ...
            string(strAssets(idxAsset).asset_id), charCanonicalName, charLocalPath, ...
            string(charManifestPath), string(charFetchCommand));
    end

    charParentFolder = string(fileparts(charLocalPath));
    if ~isfolder(charParentFolder)
        mkdir(charParentFolder);
    end

    try
        websave(charLocalPath, string(strAssets(idxAsset).download_url));
    catch objException
        error("FetchScenarioData:DownloadFailed", ...
            "Failed to download asset %s for %s from %s: %s", ...
            string(strAssets(idxAsset).asset_id), charCanonicalName, ...
            string(strAssets(idxAsset).download_url), objException.message);
    end

    strFetchAssets(idxAsset).bDownloaded = true;
end

strFetchPlan = struct( ...
    "charScenarioName", charCanonicalName, ...
    "charDataRootPath", charDataRootPath, ...
    "bDryRun", options.bDryRun, ...
    "strAssets", strFetchAssets);
end
