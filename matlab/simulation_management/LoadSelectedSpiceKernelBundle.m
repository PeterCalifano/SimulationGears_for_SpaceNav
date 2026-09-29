function strLoadedBundle = LoadSelectedSpiceKernelBundle( ...
    enumOrName, charBundleId, kwargs)
%% SIGNATURE
% strLoadedBundle = LoadSelectedSpiceKernelBundle(enumOrName, charBundleId, Name=Value)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve a named scenario kernel bundle, verify every declared file SHA-256,
% and replace the process SPICE pool with exactly its ancillary metakernel and
% trajectory. Compare the loaded kernel paths with the verified manifest set.
% The scenario manifest owns the source paths and hashes; callers select a
% bundle per run instead of changing the scenario default.
% Preserve the current pool on pre-load validation errors. Clear partial loads
% on loading/identity errors; restore the caller's folder in both cases.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% enumOrName                 Registered scenario name or enum.
% charBundleId               Kernel-bundle ID from the scenario manifest.
% kwargs.charDataRootPath    Optional tracked-manifest root override.
% kwargs.charAssetRootPath   Optional external asset-root override.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strLoadedBundle            Selected IDs, resolved paths, hashes, and
%                            verified kernel count for input provenance.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-09-2026  Pietro Califano, Codex gpt-6  Add selected, hash-checked kernel bundle loading.
% 29-09-2026  Pietro Califano     Consolidate loaded-source identity and relative-root handling.
% 29-09-2026  Pietro Califano, Codex  Use the shared MathCore file-integrity adapter.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% LoadScenarioDataManifest, ResolveScenarioAssetPath, ComputeFileSha256 (MathCore),
% MICE cspice_kclear, cspice_furnsh, cspice_ktotal, cspice_kdata;
% Java File for canonical path comparison.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    enumOrName (1, :) {mustBeA(enumOrName, ...
        ["string", "char", "EnumScenarioName"])}
    charBundleId (1, :) char {mustBeNonzeroLengthText}
    kwargs.charDataRootPath (1, :) string = ""
    kwargs.charAssetRootPath (1, :) string = ""
end
arguments (Output)
    strLoadedBundle (1, 1) struct
end

% Select exactly one manifest bundle and trajectory without changing the
% registered scenario's default metakernel.
strManifest = LoadScenarioDataManifest( ...
    enumOrName, charDataRootPath=kwargs.charDataRootPath);
if ~isfield(strManifest, 'kernel_bundles') || ...
        ~isfield(strManifest, 'external_trajectories')
    error('LoadSelectedSpiceKernelBundle:BundleUnavailable', ...
        'Scenario manifest has no selectable kernel bundles.');
end

strBundles = NormalizeManifestStructArray(strManifest.kernel_bundles);
strTrajectories = NormalizeManifestStructArray( ...
    strManifest.external_trajectories);

dBundleMatches = find(strcmp(string({strBundles.bundle_id}), ...
    string(charBundleId)));

if numel(dBundleMatches) ~= 1
    error('LoadSelectedSpiceKernelBundle:UnknownBundle', ...
        'Expected one scenario kernel bundle with ID %s.', charBundleId);
end

strBundle = strBundles(dBundleMatches);
dTrajectoryMatches = find(strcmp( ...
    string({strTrajectories.asset_id}), ...
    string(strBundle.trajectory_asset_id)));

if numel(dTrajectoryMatches) ~= 1
    error('LoadSelectedSpiceKernelBundle:UnknownTrajectory', ...
        'Bundle %s must identify one external trajectory.', charBundleId);
end

strTrajectory = strTrajectories(dTrajectoryMatches);
strAncillary = NormalizeManifestStructArray(strBundle.loaded_kernels);
if isempty(strAncillary)
    error('LoadSelectedSpiceKernelBundle:EmptyBundle', ...
        'Kernel bundle %s must declare its loaded ancillary files.', charBundleId);
end

% Verify the metakernel, each file it is expected to load, and the selected
% spacecraft trajectory before changing the current SPICE pool.
charMetaKernelFile = ValidateAsset_( ...
    strBundle.meta_kernel, kwargs.charAssetRootPath);
charTrajectoryFile = ValidateAsset_( ...
    strTrajectory, kwargs.charAssetRootPath);
cellAncillaryPaths = cell(1, numel(strAncillary));

for ui32KernelIdx = uint32(1):uint32(numel(strAncillary))
    cellAncillaryPaths{ui32KernelIdx} = ValidateAsset_( ...
        strAncillary(ui32KernelIdx), kwargs.charAssetRootPath);
end

% Load relative metakernel entries from their owning folder. Restore the
% caller's folder and clear partial loads on any loading or identity failure.
charOriginalFolder = pwd;
objFolderCleanup = onCleanup(@() cd(charOriginalFolder));

[charMetaKernelFolder, charMetaKernelName, charMetaKernelExt] = ...
    fileparts(charMetaKernelFile);
cspice_kclear();

try
    cd(charMetaKernelFolder);
    cspice_furnsh([charMetaKernelName, charMetaKernelExt]);
    cspice_furnsh(charTrajectoryFile);
    ui32LoadedCount = uint32(cspice_ktotal('ALL'));
    ui32ExpectedCount = uint32(numel(strAncillary) + 2);
    if ui32LoadedCount ~= ui32ExpectedCount
        error('LoadSelectedSpiceKernelBundle:UnexpectedKernelCount', ...
            'Loaded %u kernels; selected bundle declares %u.', ...
            ui32LoadedCount, ui32ExpectedCount);
    end

    % Compare canonical path multisets because matching counts do not prove
    % that SPICE loaded the files whose bytes were verified above.
    cellDeclaredPaths = [{charMetaKernelFile}, cellAncillaryPaths, ...
                         {charTrajectoryFile}];
    cellLoadedPaths = cell(1, double(ui32LoadedCount));

    for ui32KernelIdx = uint32(1):ui32LoadedCount
        [charLoadedFile, ~, ~, ~, bFound] = ...
            cspice_kdata(int32(ui32KernelIdx), 'ALL');
        if ~bFound
            error('LoadSelectedSpiceKernelBundle:LoadedKernelMismatch', ...
                'SPICE did not report loaded kernel %u.', ui32KernelIdx);
        end
        cellDeclaredPaths{ui32KernelIdx} = char( ...
            java.io.File(cellDeclaredPaths{ui32KernelIdx}).getCanonicalPath());
        if ~java.io.File(charLoadedFile).isAbsolute()
            % Resolve relative SPICE entries against the metakernel folder
            % because Java's process folder does not follow MATLAB cd.
            charLoadedFile = fullfile(charMetaKernelFolder, charLoadedFile);
        end
        cellLoadedPaths{ui32KernelIdx} = char( ...
            java.io.File(charLoadedFile).getCanonicalPath());
    end

    cellDeclaredPaths = sort(cellDeclaredPaths);
    cellLoadedPaths = sort(cellLoadedPaths);

    if ~isequal(cellLoadedPaths, cellDeclaredPaths)
        dFirstMismatch = find(~strcmp( ...
            cellLoadedPaths, cellDeclaredPaths), 1, 'first');
        error('LoadSelectedSpiceKernelBundle:LoadedKernelMismatch', ...
            'Verified bundle declares %s, but SPICE loaded %s.', ...
            cellDeclaredPaths{dFirstMismatch}, ...
            cellLoadedPaths{dFirstMismatch});
    end

catch objError
    cspice_kclear();
    rethrow(objError);
end

strLoadedBundle = struct( ...
    'charBundleId', charBundleId, ...
    'charScenarioName', char(strManifest.scenario_name), ...
    'charSourceRepositoryPath', char(strBundle.source_repository_path), ...
    'charMetaKernelFile', charMetaKernelFile, ...
    'charMetaKernelSha256', char(strBundle.meta_kernel.sha256), ...
    'charTrajectoryFile', charTrajectoryFile, ...
    'charTrajectorySha256', char(strTrajectory.sha256), ...
    'strTrajectoryMetadata', strTrajectory, ...
    'cellAncillaryPaths', {cellAncillaryPaths}, ...
    'cellAncillarySha256', {{strAncillary.sha256}}, ...
    'ui32LoadedKernelCount', ui32LoadedCount);
end

function charAssetFile = ValidateAsset_(strAsset, charAssetRootPath)
%% SIGNATURE
% charAssetFile = ValidateAsset_(strAsset, charAssetRootPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolve one bundle member and enforce its manifest SHA-256 before loading.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strAsset           Manifest item with local_path and sha256.
% charAssetRootPath  Optional external asset-root override.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% charAssetFile      Verified local path.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 27-09-2026  Pietro Califano, Codex gpt-6  First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ResolveScenarioAssetPath, ComputeFileSha256 (MathCore).
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strAsset (1, 1) struct
    charAssetRootPath (1, :) string
end
arguments (Output)
    charAssetFile (1, :) char
end

if ~isfield(strAsset, 'local_path') || ~isfield(strAsset, 'sha256') || ...
        isempty(char(strAsset.local_path)) || ...
        isempty(regexp(char(strAsset.sha256), '^[0-9a-fA-F]{64}$', 'once'))
    error('LoadSelectedSpiceKernelBundle:InvalidAsset', ...
        'A bundle member requires a local path and a SHA-256 digest.');
end
charAssetFile = char(ResolveScenarioAssetPath( ...
    strAsset.local_path, charAssetRootPath=charAssetRootPath));

% Resolve relative overrides against MATLAB's current folder before loading
% changes it. Java's process folder does not follow MATLAB cd.
if ~java.io.File(charAssetFile).isAbsolute()
    charAssetFile = fullfile(pwd, charAssetFile);
end
charAssetFile = char(java.io.File(charAssetFile).getCanonicalPath());
if ~isfile(charAssetFile)
    error('LoadSelectedSpiceKernelBundle:MissingAsset', ...
        'Selected kernel-bundle asset is missing: %s.', charAssetFile);
end
charActualDigest = ComputeFileSha256(charAssetFile);
if ~strcmpi(charActualDigest, char(strAsset.sha256))
    error('LoadSelectedSpiceKernelBundle:HashMismatch', ...
        'Selected kernel-bundle asset differs from its registered SHA-256: %s.', ...
        charAssetFile);
end
end
