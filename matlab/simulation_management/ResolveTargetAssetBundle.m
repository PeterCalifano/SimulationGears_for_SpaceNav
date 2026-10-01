function objAssetBundle = ResolveTargetAssetBundle(enumOrName, options)
%% SIGNATURE
% objAssetBundle = ResolveTargetAssetBundle(enumOrName, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve one manifest-selected target geometry and its appearance profile.
% All returned payload paths use the shared external asset root.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumOrName                         Scenario name or EnumScenarioName.
% options.charShapeAssetId           (1,:) string = ""; empty selects the manifest default.
% options.charAppearanceProfileId    (1,:) string = ""; empty selects the manifest default.
% options.charDataRootPath           (1,:) string = ""; tracked-manifest root override.
% options.charAssetRootPath          (1,:) string = ""; external-payload root override.
% options.bRequireFiles              (1,1) logical = true; require selected payloads to exist.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% objAssetBundle                     (1,1) CTargetAssetBundle; resolved selection.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6  First shared target-asset resolver.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% LoadScenarioDataManifest, ResolveScenarioAssetPath, CTargetAssetBundle.
% -------------------------------------------------------------------------------------------------------------

arguments
    enumOrName (1,:) {mustBeA(enumOrName, ["string", "char", "EnumScenarioName"])}
    options.charShapeAssetId (1,:) string = ""
    options.charAppearanceProfileId (1,:) string = ""
    options.charDataRootPath (1,:) string = ""
    options.charAssetRootPath (1,:) string = ""
    options.bRequireFiles (1,1) logical = true
end

[~, charCanonicalName] = CScenarioRegistry.ResolveScenario(enumOrName);
strManifest = LoadScenarioDataManifest(charCanonicalName, ...
    charDataRootPath=options.charDataRootPath);
strAssets = NormalizeManifestStructArray(strManifest.assets);

charShapeAssetId = options.charShapeAssetId;
if strlength(charShapeAssetId) == 0
    charShapeAssetId = string(strManifest.default_shape_asset_id);
end
strShapeAsset = FindAsset_(strAssets, charShapeAssetId, "shape");
charShapePath = ResolveScenarioAssetPath(string(strShapeAsset.local_path), ...
    charAssetRootPath=options.charAssetRootPath);

charShapeFormat = string(GetFieldOrDefault_(strShapeAsset, "content_format", ""));
if strlength(charShapeFormat) == 0
    [~, ~, charExtension] = fileparts(charShapePath);
    switch lower(string(charExtension))
        case ".obj"
            charShapeFormat = "obj";
        case ".bds"
            charShapeFormat = "dsk";
        otherwise
            error("ResolveTargetAssetBundle:UnknownShapeFormat", ...
                "Shape asset %s has no supported content_format.", charShapeAssetId);
    end
end

charInputUnits = string(GetFieldOrDefault_(strShapeAsset, "input_units", "km"));
enumInputUnits = EnumLengthUnits.fromAny(charInputUnits);
charBodyFixedFrame = string(GetFieldOrDefault_(strShapeAsset, "body_fixed_frame", ""));
charMaterialPath = ResolveOptionalPath_(strShapeAsset, "material_path", options.charAssetRootPath);
bLoadMaterials = logical(GetFieldOrDefault_(strShapeAsset, "load_materials", false));

charAppearanceProfileId = options.charAppearanceProfileId;
if strlength(charAppearanceProfileId) == 0 && isfield(strManifest, "default_appearance_profile_id")
    charAppearanceProfileId = string(strManifest.default_appearance_profile_id);
end

charAlbedoMapPaths = strings(1, 0);
charNormalMapPaths = strings(1, 0);
enumAlbedoMapping = EnumSurfaceMapMapping.NONE;
enumNormalMapping = EnumSurfaceMapMapping.NONE;
charAlbedoColorSpace = "";
charNormalMapEncoding = "";

if strlength(charAppearanceProfileId) > 0
    if ~isfield(strManifest, "appearance_profiles")
        error("ResolveTargetAssetBundle:MissingAppearanceProfiles", ...
            "Scenario %s selects appearance profile %s but defines none.", ...
            charCanonicalName, charAppearanceProfileId);
    end

    strProfiles = NormalizeManifestStructArray(strManifest.appearance_profiles);
    strProfileIds = string({strProfiles.profile_id});
    dProfileIdx = find(strProfileIds == charAppearanceProfileId, 1);
    if isempty(dProfileIdx)
        error("ResolveTargetAssetBundle:UnknownAppearanceProfile", ...
            "Scenario %s does not define appearance profile %s.", ...
            charCanonicalName, charAppearanceProfileId);
    end

    strProfile = strProfiles(dProfileIdx);
    charAlbedoMapPaths = ResolveAssetReferences_(strAssets, ...
        GetStringArray_(strProfile, "albedo_asset_ids"), "albedo", options.charAssetRootPath);
    charNormalMapPaths = ResolveAssetReferences_(strAssets, ...
        GetStringArray_(strProfile, "normal_asset_ids"), "normal", options.charAssetRootPath);
    enumAlbedoMapping = EnumSurfaceMapMapping.FromConfigValue( ...
        GetFieldOrDefault_(strProfile, "albedo_mapping", "NONE"));
    enumNormalMapping = EnumSurfaceMapMapping.FromConfigValue( ...
        GetFieldOrDefault_(strProfile, "normal_mapping", "NONE"));
    charAlbedoColorSpace = string(GetFieldOrDefault_(strProfile, "albedo_color_space", ""));
    charNormalMapEncoding = string(GetFieldOrDefault_(strProfile, "normal_map_encoding", ""));
end

if options.bRequireFiles
    RequireFile_(charShapePath, "shape", charShapeAssetId);
    if bLoadMaterials
        RequireFile_(charMaterialPath, "material", charShapeAssetId);
    end
    RequirePaths_(charAlbedoMapPaths, "albedo");
    RequirePaths_(charNormalMapPaths, "normal map");
end

objAssetBundle = CTargetAssetBundle(charScenarioName=string(charCanonicalName), ...
                            charShapeAssetId=charShapeAssetId, ...
                            charShapePath=charShapePath, ...
                            charShapeFormat=charShapeFormat, ...
                            enumShapeInputUnits=enumInputUnits, ...
                            charBodyFixedFrame=charBodyFixedFrame, ...
                            charShapeSha256=string(GetFieldOrDefault_(strShapeAsset, "sha256", "")), ...
                            bLoadMaterials=bLoadMaterials, ...
                            charMaterialPath=charMaterialPath, ...
                            charAppearanceProfileId=charAppearanceProfileId, ...
                            charAlbedoMapPaths=charAlbedoMapPaths, ...
                            charNormalMapPaths=charNormalMapPaths, ...
                            enumAlbedoMapping=enumAlbedoMapping, ...
                            enumNormalMapping=enumNormalMapping, ...
                            charAlbedoColorSpace=charAlbedoColorSpace, ...
                            charNormalMapEncoding=charNormalMapEncoding);
end

function strAsset = FindAsset_(strAssets, charAssetId, charExpectedType)
strAssetIds = string({strAssets.asset_id});
dAssetIdx = find(strAssetIds == charAssetId, 1);
if isempty(dAssetIdx)
    error("ResolveTargetAssetBundle:UnknownAsset", ...
        "Asset %s is not present in the scenario manifest.", charAssetId);
end

strAsset = strAssets(dAssetIdx);
if string(strAsset.asset_type) ~= charExpectedType
    error("ResolveTargetAssetBundle:AssetTypeMismatch", ...
        "Asset %s has type %s, expected %s.", ...
        charAssetId, string(strAsset.asset_type), charExpectedType);
end
end

function varValue = GetFieldOrDefault_(strInput, charFieldName, varDefault)
if isfield(strInput, charFieldName)
    varValue = strInput.(charFieldName);
else
    varValue = varDefault;
end
end

function charValues = GetStringArray_(strInput, charFieldName)
charValues = strings(1, 0);
if isfield(strInput, charFieldName)
    charValues = reshape(string(strInput.(charFieldName)), 1, []);
end
end

function charResolvedPath = ResolveOptionalPath_(strInput, charFieldName, charAssetRootPath)
charResolvedPath = "";
charRelativePath = string(GetFieldOrDefault_(strInput, charFieldName, ""));
if strlength(charRelativePath) > 0
    charResolvedPath = ResolveScenarioAssetPath(charRelativePath, ...
        charAssetRootPath=charAssetRootPath);
end
end

function charPaths = ResolveAssetReferences_(strAssets, charAssetIds, charExpectedType, charAssetRootPath)
charPaths = strings(1, numel(charAssetIds));
for dAssetIdx = 1:numel(charAssetIds)
    strAsset = FindAsset_(strAssets, charAssetIds(dAssetIdx), charExpectedType);
    charPaths(dAssetIdx) = ResolveScenarioAssetPath(string(strAsset.local_path), ...
        charAssetRootPath=charAssetRootPath);
end
end

function RequirePaths_(charPaths, charRole)
for dPathIdx = 1:numel(charPaths)
    RequireFile_(charPaths(dPathIdx), charRole, "");
end
end

function RequireFile_(charPath, charRole, charAssetId)
if strlength(charPath) == 0 || ~isfile(charPath)
    error("ResolveTargetAssetBundle:MissingAssetFile", ...
        "Required %s asset %s was not found at %s.", ...
        charRole, charAssetId, charPath);
end
end
