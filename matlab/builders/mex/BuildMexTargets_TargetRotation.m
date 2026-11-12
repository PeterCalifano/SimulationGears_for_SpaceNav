function strMexInfo = BuildMexTargets_TargetRotation(charOutputRoot)
%% SIGNATURE
% strMexInfo = BuildMexTargets_TargetRotation(charOutputRoot)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build or reuse one source-fingerprinted whole-history torque-free target-rotation MEX profile. Generated code
% is produced in a private temporary directory and the complete binary, report, and metadata directory is
% published by rename, so selectors never accept a partially generated artifact.
%
% The profile identity includes the complete resolved MATLAB source closure, builder source, MATLAB release, MEX
% extension, and fixed public gateway contract. Runtime timegrid length and physical values remain variable
% inputs and therefore do not create redundant profiles.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot    Optional artifact root. Empty selects matlab/mex/simulation_models/attitude.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strMexInfo        Struct containing profile identity, exact artifact paths, target name, and build/reuse
%                   status.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 23-08-2026  Pietro Califano, Codex     First whole-history target-rotation MEX builder.
% 24-08-2026  Pietro Califano, Codex     Add source-fingerprinted reuse and atomic profile publication.
% 11-11-2026  Pietro Califano, Claude Opus 5.5   Readability pass.
% 12-11-2026  Pietro Califano, Claude Opus 5.5   Hash with MathCore ComputeFileSha256 and DataHash.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% PropagateTorqueFreeBodyHistory, ResolveMexBuildDirectory,
% matlab.codetools.requiredFilesAndProducts, ComputeFileSha256, DataHash.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    charOutputRoot {mustBeTextScalar} = ""
end
arguments (Output)
    strMexInfo (1,1) struct
end

% Resolve the generated-artifact root only after MEX execution is selected by the caller. MATLAB reference
% execution never calls this builder.
charOutputRoot = ResolveMexBuildDirectory(charOutputRoot, 'simulation_models', 'attitude');
strProfile = BuildProfile_(charOutputRoot);

% Reuse only a complete artifact whose metadata matches every element of the current profile contract.
if CheckArtifact_(strProfile)
    strMexInfo = BuildInfo_(strProfile, false);
    return
end

% Compile in a unique sibling directory. An onCleanup guard removes failed or superseded output without
% touching any published profile.
charTemporaryProfileDir = tempname(charOutputRoot);
mkdir(charTemporaryProfileDir);
objTemporaryCleanup = onCleanup(@() RemoveFolderIfPresent_(charTemporaryProfileDir));

% Configure a MEX build with a report and variable-size arrays, and a variable-length timegrid
objCodegenConfig = coder.config('mex');
objCodegenConfig.GenerateReport = true;
if isprop(objCodegenConfig, 'EnableDynamicMemoryAllocation')
    objCodegenConfig.EnableDynamicMemoryAllocation = true;
elseif isprop(objCodegenConfig, 'DynamicMemoryAllocation')
    objCodegenConfig.DynamicMemoryAllocation = 'AllVariableSizeArrays';
end

objTimegridType = coder.typeof(0.0, [1, Inf], [false, true]);
cellCodegenArgs = {objTimegridType, zeros(4, 1), zeros(3, 1), eye(3), 0.1};

% Generate the MEX inside the temporary directory and return to the caller folder afterwards
charCallerDirectory = pwd;
objDirectoryCleanup = onCleanup(@() cd(charCallerDirectory));
try
    cd(charTemporaryProfileDir);
    codegen('-config', objCodegenConfig, ...
        '-d', charTemporaryProfileDir, ...
        '-o', strProfile.charTargetFunctionName, ...
        'PropagateTorqueFreeBodyHistory', ...
        '-args', cellCodegenArgs);
catch objBuildException
    fprintf(2, ...
        ['Code generation failed while building target-rotation profile %s. ', ...
        'No artifact was published.\n'], ...
        strProfile.charProfileKey);
    fprintf(2, '%s\n', objBuildException.message);
    rethrow(objBuildException);
end
cd(charCallerDirectory);

% Verify the binary landed at the public profile path before metadata makes the temporary directory eligible
% for publication.
charTemporaryArtifactPath = fullfile(charTemporaryProfileDir, [strProfile.charTargetFunctionName, '.', mexext]);
if ~isfile(charTemporaryArtifactPath)
    error('BuildMexTargets_TargetRotation:GeneratedArtifactNotFound', ...
        'Generated target was not found at %s.', charTemporaryArtifactPath);
end

% Write the profile metadata with the build time
strArtifactMetadata = strProfile;
strArtifactMetadata.charBuildTimestamp = char(datetime('now', 'TimeZone', 'UTC'));
save(fullfile(charTemporaryProfileDir, 'target_rotation_mex_profile.mat'), 'strArtifactMetadata');

% A concurrent builder may have published the same exact profile while this process compiled. Prefer that
% complete artifact over replacing it.
if CheckArtifact_(strProfile)
    strMexInfo = BuildInfo_(strProfile, false);
    return
end

% Publish the new profile and check it
ReplaceDirectory_(charTemporaryProfileDir, strProfile.charArtifactDir);
if ~CheckArtifact_(strProfile)
    error('BuildMexTargets_TargetRotation:PublicationFailed', ...
        'Published target-rotation MEX profile failed validation.');
end

strMexInfo = BuildInfo_(strProfile, true);
end

function strProfile = BuildProfile_(charOutputRoot)
%% SIGNATURE
% strProfile = BuildProfile_(charOutputRoot)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Derive one path-independent source and gateway identity for the generated target.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charOutputRoot   Resolved artifact root.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strProfile       Profile key, target name, artifact paths and the contracts with their hashes.
% -------------------------------------------------------------------------------------------------------------

% Collect the entry point, its resolved source closure and this builder
charEntryPointPath = which('PropagateTorqueFreeBodyHistory');
if isempty(charEntryPointPath)
    error('BuildMexTargets_TargetRotation:MissingEntryPoint', ...
        'PropagateTorqueFreeBodyHistory is not on the MATLAB path.');
end

cellRequiredFiles = matlab.codetools.requiredFilesAndProducts(charEntryPointPath);
charBuilderPath = [mfilename('fullpath'), '.m'];
cellRequiredFiles = unique([{charEntryPointPath}; cellRequiredFiles(:); {charBuilderPath}], 'stable');

% Hash each source file by name and content, independent of its folder and of the file order
cellSourceEntries = cell(numel(cellRequiredFiles), 1);
for ui32FileIdx = uint32(1):uint32(numel(cellRequiredFiles))
    charSourcePath = cellRequiredFiles{double(ui32FileIdx)};
    if ~isfile(charSourcePath)
        error('BuildMexTargets_TargetRotation:MissingSourceDependency', ...
            'Resolved source dependency is absent: %s.', charSourcePath);
    end
    [~, charSourceName, charSourceExtension] = fileparts(charSourcePath);
    cellSourceEntries{double(ui32FileIdx)} = sprintf('%s%s:%s', ...
        charSourceName, ...
        charSourceExtension, ...
        ComputeFileSha256(charSourcePath));
end
cellSourceEntries = sort(cellSourceEntries);
charSourceContract = strjoin(cellSourceEntries, '|');
charSourceFingerprint = DataHash(unicode2native(charSourceContract, 'UTF-8'), 'SHA-256', 'bin', 'hex');

% Combine target, release, MEX extension, gateway and source fingerprint into the profile key
charTargetFunctionName = 'PropagateTorqueFreeBodyHistory_mex';
charGatewayContract = 'double[1xN]|double[4x1]|double[3x1]|double[3x3]|double[1x1]';
charProfileContract = sprintf('target=%s|release=%s|mex=%s|gateway=%s|source=%s', ...
    charTargetFunctionName, ...
    version('-release'), ...
    mexext, ...
    charGatewayContract, ...
    charSourceFingerprint);
charContractHash = DataHash(unicode2native(charProfileContract, 'UTF-8'), 'SHA-256', 'bin', 'hex');
charProfileKey = sprintf('torque_free_%s', charContractHash(1:16));
charArtifactDir = fullfile(charOutputRoot, charProfileKey);

strProfile = struct();
strProfile.charProfileKey = charProfileKey;
strProfile.charTargetFunctionName = charTargetFunctionName;
strProfile.charOutputRoot = charOutputRoot;
strProfile.charArtifactDir = charArtifactDir;
strProfile.charArtifactPath = fullfile(charArtifactDir, [charTargetFunctionName, '.', mexext]);
strProfile.charMetadataPath = fullfile(charArtifactDir, 'target_rotation_mex_profile.mat');
strProfile.charMatlabRelease = version('-release');
strProfile.charMexExtension = mexext;
strProfile.charGatewayContract = charGatewayContract;
strProfile.charSourceContract = charSourceContract;
strProfile.charSourceFingerprint = charSourceFingerprint;
strProfile.charProfileContract = charProfileContract;
strProfile.charContractHash = charContractHash;
end

function bIsCurrent = CheckArtifact_(strProfile)
%% SIGNATURE
% bIsCurrent = CheckArtifact_(strProfile)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Accept an artifact only when its binary and complete contract metadata match.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strProfile    Profile from BuildProfile_.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bIsCurrent    True when the published binary and metadata match every contract field.
% -------------------------------------------------------------------------------------------------------------

% Require both the binary and a loadable scalar metadata struct
bIsCurrent = false;
if ~isfile(strProfile.charArtifactPath) || ~isfile(strProfile.charMetadataPath)
    return
end

try
    strLoaded = load(strProfile.charMetadataPath, 'strArtifactMetadata');
catch
    return
end
if ~isfield(strLoaded, 'strArtifactMetadata') || ...
        ~isstruct(strLoaded.strArtifactMetadata) || ...
        ~isscalar(strLoaded.strArtifactMetadata)
    return
end

% Compare every contract field with the current profile
strMetadata = strLoaded.strArtifactMetadata;
cellContractFields = {'charProfileKey', ...
    'charTargetFunctionName', ...
    'charMatlabRelease', ...
    'charMexExtension', ...
    'charGatewayContract', ...
    'charSourceFingerprint', ...
    'charProfileContract', ...
    'charContractHash'};
for ui32FieldIdx = uint32(1):uint32(numel(cellContractFields))
    charFieldName = cellContractFields{double(ui32FieldIdx)};
    if ~isfield(strMetadata, charFieldName) || ~strcmp(strMetadata.(charFieldName), strProfile.(charFieldName))
        return
    end
end
bIsCurrent = true;
end

function strMexInfo = BuildInfo_(strProfile, bArtifactBuilt)
%% SIGNATURE
% strMexInfo = BuildInfo_(strProfile, bArtifactBuilt)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Return stable builder and selector information without duplicating profile fields.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strProfile       Profile from BuildProfile_.
% bArtifactBuilt   True when this call compiled and published the artifact.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strMexInfo       Profile fields plus cellMexTargets and bArtifactBuilt.
% -------------------------------------------------------------------------------------------------------------

strMexInfo = strProfile;
strMexInfo.cellMexTargets = {strProfile.charTargetFunctionName};
strMexInfo.bArtifactBuilt = bArtifactBuilt;
end

function ReplaceDirectory_(charSourceDir, charTargetDir)
%% SIGNATURE
% ReplaceDirectory_(charSourceDir, charTargetDir)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Publish one complete generated profile through sibling directory renames. A stale target is moved aside
% first and restored when the new profile cannot be moved in.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charSourceDir   Temporary profile directory.
% charTargetDir   Published profile directory.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------

% Move a stale profile aside
charTargetParent = fileparts(charTargetDir);
charBackupDir = '';
if isfolder(charTargetDir)
    charBackupDir = tempname(charTargetParent);
    [bMovedOld, charMoveOldMessage] = movefile(charTargetDir, charBackupDir);
    if ~bMovedOld
        error('BuildMexTargets_TargetRotation:PublishRenameFailed', ...
            'Unable to move the stale generated profile: %s', ...
            charMoveOldMessage);
    end
end

% Move the new profile in, restoring the stale one on failure
[bMovedNew, charMoveNewMessage] = movefile(charSourceDir, charTargetDir);
if ~bMovedNew
    if ~isempty(charBackupDir) && isfolder(charBackupDir)
        movefile(charBackupDir, charTargetDir);
    end
    error('BuildMexTargets_TargetRotation:PublishRenameFailed', ...
        'Unable to publish the generated profile: %s', ...
        charMoveNewMessage);
end

if ~isempty(charBackupDir)
    RemoveFolderIfPresent_(charBackupDir);
end
end

function RemoveFolderIfPresent_(charFolderPath)
%% SIGNATURE
% RemoveFolderIfPresent_(charFolderPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Remove only an unpublished or replaced generated profile directory.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charFolderPath   Directory to remove when it exists.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None.
% -------------------------------------------------------------------------------------------------------------

if isfolder(charFolderPath)
    rmdir(charFolderPath, 's');
end
end
