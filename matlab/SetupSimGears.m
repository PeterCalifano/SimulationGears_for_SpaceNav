function SetupSimGears()
%% DESCRIPTION
% Add all SimulationGears MATLAB source directories to the MATLAB path.
% Can be called from any working directory. Excludes .deprecated/, generated
% MEX artifacts, codegen/mex/, and experimental/ folders.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-03-2026    Pietro Califano     Initial version
% 27-07-2026    Pietro Califano, Codex     Prioritize shared SimulationGears propagators.
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

% Add MathCore MATLAB library (sibling of matlab/ under repo root)
charRepoRoot    = fileparts(charMatlabRoot);
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
