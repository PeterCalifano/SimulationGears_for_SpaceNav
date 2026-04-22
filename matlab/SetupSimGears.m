function SetupSimGears()
%% DESCRIPTION
% Add all SimulationGears MATLAB source directories to the MATLAB path.
% Can be called from any working directory. Excludes .deprecated/, codegen/mex/,
% and experimental/ folders.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-03-2026    Pietro Califano     Initial version
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

% Resolve the matlab/ root from the location of this script
charMatlabRoot = fileparts(mfilename('fullpath'));

% Generate all subdirectories
cellAllPaths = strsplit(genpath(charMatlabRoot), pathsep);

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

% Add MathCore_for_SpaceNav library (sibling of matlab/ under repo root)
charRepoRoot    = fileparts(charMatlabRoot);
charMathCoreRoot = fullfile(charRepoRoot, 'lib', 'MathCore_for_SpaceNav', 'matlab');

if exist(charMathCoreRoot, 'dir')
    cellMathCorePaths = strsplit(genpath(charMathCoreRoot), pathsep);

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
    fprintf('MathCore_for_SpaceNav MATLAB paths added from: %s\n', charMathCoreRoot);
else
    warning('SetupSimGears:MathCoreNotFound', ...
        'MathCore_for_SpaceNav not found at expected location: %s', charMathCoreRoot);
end
end
