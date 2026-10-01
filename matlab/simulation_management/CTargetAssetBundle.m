classdef CTargetAssetBundle
%% DESCRIPTION
% Resolved target geometry and appearance selected from one scenario manifest.
% The bundle stores asset identity and paths; CShapeModel owns loaded geometry.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6  First shared truth-render asset bundle.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EnumLengthUnits, EnumSurfaceMapMapping.
% -------------------------------------------------------------------------------------------------------------

properties (SetAccess = immutable)
    charScenarioName (1,1) string
    charShapeAssetId (1,1) string
    charShapePath (1,1) string
    charShapeFormat (1,1) string
    enumShapeInputUnits (1,1) EnumLengthUnits
    charBodyFixedFrame (1,1) string
    charShapeSha256 (1,1) string
    bLoadMaterials (1,1) logical
    charMaterialPath (1,1) string
    charAppearanceProfileId (1,1) string
    charAlbedoMapPaths (1,:) string
    charNormalMapPaths (1,:) string
    enumAlbedoMapping (1,1) EnumSurfaceMapMapping
    enumNormalMapping (1,1) EnumSurfaceMapMapping
    charAlbedoColorSpace (1,1) string
    charNormalMapEncoding (1,1) string
end

methods
    function self = CTargetAssetBundle(options)
        %% SIGNATURE
        % self = CTargetAssetBundle(options)
        % -----------------------------------------------------------------------------------------------------
        %% DESCRIPTION
        % Construct one validated target-asset selection.
        % -----------------------------------------------------------------------------------------------------
        %% INPUT
        % options.*   Resolved geometry and appearance metadata.
        % -----------------------------------------------------------------------------------------------------
        %% OUTPUT
        % self        (1,1) CTargetAssetBundle.
        % -----------------------------------------------------------------------------------------------------
        %% CHANGELOG
        % 21-09-2026  Pietro Califano, Codex gpt-5.6  First implementation.
        % -----------------------------------------------------------------------------------------------------
        %% DEPENDENCIES
        % EnumLengthUnits, EnumSurfaceMapMapping.
        % -----------------------------------------------------------------------------------------------------
        arguments
            options.charScenarioName (1,1) string
            options.charShapeAssetId (1,1) string
            options.charShapePath (1,1) string
            options.charShapeFormat (1,1) string
            options.enumShapeInputUnits (1,1) EnumLengthUnits
            options.charBodyFixedFrame (1,1) string
            options.charShapeSha256 (1,1) string = ""
            options.bLoadMaterials (1,1) logical = false
            options.charMaterialPath (1,1) string = ""
            options.charAppearanceProfileId (1,1) string = ""
            options.charAlbedoMapPaths (1,:) string = strings(1, 0)
            options.charNormalMapPaths (1,:) string = strings(1, 0)
            options.enumAlbedoMapping (1,1) EnumSurfaceMapMapping = EnumSurfaceMapMapping.NONE
            options.enumNormalMapping (1,1) EnumSurfaceMapMapping = EnumSurfaceMapMapping.NONE
            options.charAlbedoColorSpace (1,1) string = ""
            options.charNormalMapEncoding (1,1) string = ""
        end

        self.charScenarioName = options.charScenarioName;
        self.charShapeAssetId = options.charShapeAssetId;
        self.charShapePath = options.charShapePath;
        self.charShapeFormat = lower(options.charShapeFormat);
        self.enumShapeInputUnits = options.enumShapeInputUnits;
        self.charBodyFixedFrame = options.charBodyFixedFrame;
        self.charShapeSha256 = lower(options.charShapeSha256);
        self.bLoadMaterials = options.bLoadMaterials;
        self.charMaterialPath = options.charMaterialPath;
        self.charAppearanceProfileId = options.charAppearanceProfileId;
        self.charAlbedoMapPaths = options.charAlbedoMapPaths;
        self.charNormalMapPaths = options.charNormalMapPaths;
        self.enumAlbedoMapping = options.enumAlbedoMapping;
        self.enumNormalMapping = options.enumNormalMapping;
        self.charAlbedoColorSpace = options.charAlbedoColorSpace;
        self.charNormalMapEncoding = options.charNormalMapEncoding;
    end
end
end
