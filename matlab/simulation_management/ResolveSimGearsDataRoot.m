function charDataRootPath = ResolveSimGearsDataRoot(options)
%% DESCRIPTION
% Resolve the SimulationGears-owned data root with fail-fast validation.
% -------------------------------------------------------------------------------------------------------------
%% SIGNATURE
% charDataRootPath = ResolveSimGearsDataRoot(options)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% options.charDataRootPath (1,:) string = ""; explicit data-root override.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% charDataRootPath         (1,:) string; existing SimulationGears data root.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 01-07-2026    Pietro Califano     Add SimulationGears-owned data-root resolution.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% getenv, mfilename, isfolder.
% -------------------------------------------------------------------------------------------------------------

arguments
    options.charDataRootPath (1,:) string = ""
end

if strlength(options.charDataRootPath) > 0
    charDataRootPath = string(options.charDataRootPath);
elseif strlength(string(getenv("SIMGEARS_DATA_ROOT"))) > 0
    charDataRootPath = string(getenv("SIMGEARS_DATA_ROOT"));
else
    charThisFile = string(mfilename("fullpath"));
    charRepoRoot = fileparts(fileparts(fileparts(charThisFile)));
    charDataRootPath = string(fullfile(charRepoRoot, "data"));
end

if ~isfolder(charDataRootPath)
    error("ResolveSimGearsDataRoot:MissingDataRoot", ...
        "SimulationGears data root not found at %s. Create it or set SIMGEARS_DATA_ROOT.", ...
        charDataRootPath);
end
end
