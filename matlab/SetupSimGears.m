function SetupSimGears()
%% DESCRIPTION
% Add all SimulationGears MATLAB source directories to the MATLAB path.
% Can be called from any working directory. Excludes .deprecated/, generated
% MEX artifacts, codegen/mex/, and experimental/ folders.
% Optional external dependencies are added only when their environment variable is set:
%   SIM_GEARS_CAMERA_NOISE_MEX_DIR  camera-noise MEX installation (independent of any renderer);
%   SIM_GEARS_SPECTRA_RT_DIR        Spectra-RT build root; adds wrap/spectra_rt and wrap/spectra_rt_mex;
%   SIM_GEARS_COMM_MANAGER_DIR      CommManager4MATLAB root; adds src and its subfolders at the end.
% The vendored lib/yaml package is added when no other yaml package is on the path.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-03-2026    Pietro Califano     Initial version
% 27-07-2026    Pietro Califano, Codex     Prioritize shared SimulationGears propagators.
% 11-11-2026    Pietro Califano, Claude Opus 5.5  Record the optional camera-noise MEX directory.
% 11-11-2026    Pietro Califano, Claude Opus 5.5  Add opt-in Spectra-RT and CommManager4MATLAB directories
%                                                 and the yaml fallback.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

% Resolve the matlab/ root from the location of this script
charMatlabRoot = fileparts(mfilename('fullpath'));

% Generate all subdirectories
cellAllPaths = strsplit(genpath(charMatlabRoot), pathsep);
charGeneratedMexRoot = fullfile(charMatlabRoot, 'mex');

% Filter out unwanted directories
cellExcludePatterns = {'.deprecated', ...
    fullfile('codegen', 'mex'), ...
    fullfile('experimental'), ...
    '.programs'};

for idP = 1:length(cellAllPaths)
    charPath = cellAllPaths{idP};

    if isempty(charPath)
        continue;
    end

    bExclude = false;
    if strcmp(charPath, charGeneratedMexRoot) || startsWith(charPath, [charGeneratedMexRoot filesep])
        bExclude = true;
    end

    for idE = 1:length(cellExcludePatterns)
        if contains(charPath, cellExcludePatterns{idE})
            bExclude = true;
            break;
        end
    end

    if ~bExclude
        addpath(charPath);
    end
end

fprintf('SimulationGears MATLAB paths added from: %s\n', charMatlabRoot);

% Admit only explicitly chosen external installations, never every generated build.
AddDirectoriesFromEnv_('SIM_GEARS_CAMERA_NOISE_MEX_DIR', {''}, ...
    'SetupSimGears:MissingCameraNoiseMexDirectory', '-begin', false);
AddDirectoriesFromEnv_('SIM_GEARS_SPECTRA_RT_DIR', ...
    {fullfile('wrap', 'spectra_rt'), fullfile('wrap', 'spectra_rt_mex')}, ...
    'SetupSimGears:MissingSpectraRtDirectory', '-begin', false);

% Put CommManager4MATLAB last so its utilities never shadow SimulationGears functions
AddDirectoriesFromEnv_('SIM_GEARS_COMM_MANAGER_DIR', {'src'}, ...
    'SetupSimGears:MissingCommManagerDirectory', '-end', true);

% Add the vendored yaml package unless another installation already provides it
charRepoRoot = fileparts(charMatlabRoot);
charYamlRoot = fullfile(charRepoRoot, 'lib', 'yaml');
if isempty(which('yaml.dumpFile')) && isfolder(charYamlRoot)
    addpath(charYamlRoot);
end

% Add MathCore MATLAB library (sibling of matlab/ under repo root)
cellMathCoreRootCandidates = { ...
    fullfile(charRepoRoot, 'lib', 'MathCore_for_ComputerVision', 'matlab'), ...
    fullfile(charRepoRoot, 'lib', 'MathCore_for_SpaceNav', 'matlab')};

charMathCoreRoot = "";
for idCandidate = 1:numel(cellMathCoreRootCandidates)
    charCandidateRoot = cellMathCoreRootCandidates{idCandidate};
    if exist(charCandidateRoot, 'dir')
        charMathCoreRoot = string(charCandidateRoot);
        break;
    end
end

if strlength(charMathCoreRoot) > 0
    cellMathCorePaths = strsplit(genpath(char(charMathCoreRoot)), pathsep);

    cellMathCoreExclude = {'.deprecated', fullfile('codegen', 'mex'), 'experimental'};

    for idP = 1:length(cellMathCorePaths)
        charPath = cellMathCorePaths{idP};
        if isempty(charPath)
            continue;
        end
        bExclude = false;
        for idE = 1:length(cellMathCoreExclude)
            if contains(charPath, cellMathCoreExclude{idE})
                bExclude = true;
                break;
            end
        end
        if ~bExclude
            addpath(charPath);
        end
    end
    fprintf('MathCore MATLAB paths added from: %s\n', char(charMathCoreRoot));
else
    warning('SetupSimGears:MathCoreNotFound', ...
        'MathCore MATLAB sources not found under expected lib/MathCore submodules.');
end

% Keep both levels of the SimulationGears provider authoritative while the
% migrated legacy MathCore implementations await their removal batch.
charPropagatorRoot = fullfile(charMatlabRoot, ...
    'simulation_models', 'propagators');
charIntegratorRoot = fullfile(charPropagatorRoot, 'integrators');
if isfolder(charPropagatorRoot)
    addpath(charPropagatorRoot, '-begin');
end
if isfolder(charIntegratorRoot)
    addpath(charIntegratorRoot, '-begin');
end
end

function AddDirectoriesFromEnv_(charVariableName, cellRelativeFolders, charErrorId, charPosition, bRecursive)
%% SIGNATURE
% AddDirectoriesFromEnv_(charVariableName, cellRelativeFolders, charErrorId, charPosition, bRecursive)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Add folders under the directory named by an environment variable. Do nothing when the variable is
% unset; raise charErrorId when a listed folder is missing.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charVariableName     Environment variable that holds the root directory.
% cellRelativeFolders  Folders relative to that root; '' is the root itself.
% charErrorId          Error identifier for a missing folder.
% charPosition         '-begin' or '-end' of the MATLAB path.
% bRecursive           Also add every subfolder.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Share the optional-directory handling.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charVariableName (1,:) char
    cellRelativeFolders (1,:) cell
    charErrorId (1,:) char
    charPosition (1,:) char {mustBeMember(charPosition, {'-begin', '-end'})}
    bRecursive (1,1) logical
end

charRoot = getenv(charVariableName);
if isempty(charRoot)
    return
end

% Check every folder before changing the path
cellFolders = fullfile(charRoot, cellRelativeFolders);
for ui32FolderIdx = uint32(1):uint32(numel(cellFolders))
    if ~isfolder(cellFolders{ui32FolderIdx})
        error(charErrorId, '%s does not contain the expected directory: %s', ...
            charVariableName, cellFolders{ui32FolderIdx});
    end
end

for ui32FolderIdx = uint32(1):uint32(numel(cellFolders))
    charFolder = cellFolders{ui32FolderIdx};
    if bRecursive
        charFolder = genpath(charFolder);
    end
    addpath(charFolder, charPosition);
end
end
