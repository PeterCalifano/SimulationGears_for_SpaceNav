function strManifest = ValidateScenarioDataManifest(strManifest, enumOrName, options)
%% DESCRIPTION
% Validate a SimulationGears scenario data manifest before asset use.
% -------------------------------------------------------------------------------------------------------------
%% SIGNATURE
% strManifest = ValidateScenarioDataManifest(strManifest, enumOrName, options)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strManifest                         (1,1) struct decoded from a scenario manifest JSON file.
% enumOrName                          (1,:) string, char, or EnumScenarioName identifying the expected scenario.
% options.charDataRootPath            (1,:) string = ""; override for the SimulationGears data root.
% options.bRequireLocalAssets         (1,1) logical = false; require local shape-runnable assets to exist.
% options.dMaxPreferredShapeAssetSizeGB (1,1) double = 1.0; preferred upper bound for default shape assets.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strManifest                         (1,1) validated manifest struct, unchanged.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-07-2026    Pietro Califano     Add manifest schema and registry-consistency validation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CScenarioRegistry, ResolveSimGearsDataRoot.
% -------------------------------------------------------------------------------------------------------------

arguments
    strManifest (1,1) struct
    enumOrName (1,:) {mustBeA(enumOrName, ["string", "char", "EnumScenarioName"])}
    options.charDataRootPath (1,:) string = ""
    options.bRequireLocalAssets (1,1) logical = false
    options.dMaxPreferredShapeAssetSizeGB (1,1) double {mustBePositive} = 1.0
end

[~, charCanonicalName] = CScenarioRegistry.ResolveScenario(enumOrName);
strScenarioSpec = CScenarioRegistry.GetScenarioSpec(charCanonicalName);
RequireFields_(strManifest, ["schema_version", "scenario_name", "canonical_name", ...
    "aliases", "confidence", "tags", "default_shape_asset_id", "assets"]);

if string(strManifest.scenario_name) ~= charCanonicalName || string(strManifest.canonical_name) ~= charCanonicalName
    error("ValidateScenarioDataManifest:ScenarioMismatch", ...
        "Manifest declares scenario %s/%s but %s was requested.", ...
        string(strManifest.scenario_name), string(strManifest.canonical_name), charCanonicalName);
end

if isempty(strManifest.assets)
    error("ValidateScenarioDataManifest:MissingAssets", ...
        "Manifest for %s must define at least one asset.", charCanonicalName);
end

strAssets = strManifest.assets;
RequireFields_(strAssets, ["asset_id", "asset_type", "local_path", "source_url", ...
    "download_url", "sha256", "size_gb", "fidelity", "required_for_shape_runnable"]);

strAssetIds = string({strAssets.asset_id});
charDefaultShapeAssetId = string(strManifest.default_shape_asset_id);
if strlength(string(strScenarioSpec.charDefaultShapeAssetId)) > 0 && ...
        charDefaultShapeAssetId ~= string(strScenarioSpec.charDefaultShapeAssetId)
    error("ValidateScenarioDataManifest:DefaultShapeMismatch", ...
        "Manifest for %s defaults to %s, but the registry defaults to %s.", ...
        charCanonicalName, charDefaultShapeAssetId, string(strScenarioSpec.charDefaultShapeAssetId));
end

idxDefaultShape = find(strAssetIds == charDefaultShapeAssetId, 1);
if isempty(idxDefaultShape)
    error("ValidateScenarioDataManifest:MissingDefaultShape", ...
        "Manifest for %s default shape asset %s is not present in assets.", ...
        charCanonicalName, charDefaultShapeAssetId);
end

strDefaultShapeAsset = strAssets(idxDefaultShape);
if string(strDefaultShapeAsset.asset_type) ~= "shape"
    error("ValidateScenarioDataManifest:InvalidDefaultShape", ...
        "Manifest for %s default asset %s is type %s, expected shape.", ...
        charCanonicalName, charDefaultShapeAssetId, string(strDefaultShapeAsset.asset_type));
end

if any(strcmp(string(strManifest.tags), "shape_runnable")) && ...
        ~logical(strDefaultShapeAsset.required_for_shape_runnable)
    error("ValidateScenarioDataManifest:DefaultShapeNotRunnable", ...
        "Manifest for %s default shape asset %s must be required_for_shape_runnable.", ...
        charCanonicalName, charDefaultShapeAssetId);
end

if strlength(string(strScenarioSpec.charDefaultShapeRelativePath)) > 0 && ...
        string(strDefaultShapeAsset.local_path) ~= string(strScenarioSpec.charDefaultShapeRelativePath)
    error("ValidateScenarioDataManifest:DefaultShapePathMismatch", ...
        "Manifest for %s default asset %s uses local_path %s, but the registry expects %s.", ...
        charCanonicalName, charDefaultShapeAssetId, string(strDefaultShapeAsset.local_path), ...
        string(strScenarioSpec.charDefaultShapeRelativePath));
end

if double(strDefaultShapeAsset.size_gb) > options.dMaxPreferredShapeAssetSizeGB
    error("ValidateScenarioDataManifest:DefaultShapeTooLarge", ...
        "Default shape asset %s is %.3g GB, above the %.3g GB preferred limit.", ...
        charDefaultShapeAssetId, double(strDefaultShapeAsset.size_gb), ...
        options.dMaxPreferredShapeAssetSizeGB);
end

if options.bRequireLocalAssets
    charDataRootPath = ResolveSimGearsDataRoot(charDataRootPath=options.charDataRootPath);
    charManifestPath = fullfile(charDataRootPath, strScenarioSpec.charDataManifestRelativePath);

    for idxAsset = 1:numel(strAssets)

        if isfield(strAssets(idxAsset), "required_for_shape_runnable") && ...
                logical(strAssets(idxAsset).required_for_shape_runnable)

            charAssetPath = fullfile(charDataRootPath, string(strAssets(idxAsset).local_path));

            if ~isfile(charAssetPath) && ~isfolder(charAssetPath)
                charFetchCommand = sprintf('python3 tools/data/fetch_scenario_assets.py --scenario %s --asset-id %s', ...
                    charCanonicalName, string(strAssets(idxAsset).asset_id));
                error("ValidateScenarioDataManifest:MissingAsset", ...
                    ['Required local asset %s for %s was not found.\n' ...
                     'Expected: %s\n' ...
                     'Manifest: %s\n' ...
                     'Data root: %s\n\n' ...
                     'Fetch it with:\n' ...
                     '  %s'], ...
                    string(strAssets(idxAsset).asset_id), charCanonicalName, string(charAssetPath), ...
                    string(charManifestPath), string(charDataRootPath), string(charFetchCommand));
            end

        end

    end
end
end

function RequireFields_(strInput, cellRequiredFields)
%% DESCRIPTION
% Assert required manifest fields are present on a scalar or struct array.
% -------------------------------------------------------------------------------------------------------------
cellPresentFields = string(fieldnames(strInput));
cellMissingFields = string(cellRequiredFields(~ismember(cellRequiredFields, cellPresentFields)));
if ~isempty(cellMissingFields)
    error("ValidateScenarioDataManifest:InvalidSchema", ...
        "Manifest is missing required field(s): %s.", strjoin(cellMissingFields, ", "));
end
end
