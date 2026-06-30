function AssertNoRootMexArtifacts(charRepoRoot)
%% PROTOTYPE
% AssertNoRootMexArtifacts(charRepoRoot)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Fails when generated MEX artifacts are present directly in the repository root.
% -------------------------------------------------------------------------------------------------------------

if nargin < 1 || strlength(string(charRepoRoot)) == 0
    charGuardDir = fileparts(mfilename('fullpath'));
    charRepoRoot = fileparts(fileparts(fileparts(charGuardDir)));
else
    charRepoRoot = char(string(charRepoRoot));
end

strRootMexFiles = dir(fullfile(charRepoRoot, '*.mex*'));
strRootMexFiles = strRootMexFiles(~[strRootMexFiles.isdir]);

if ~isempty(strRootMexFiles)
    cellNames = {strRootMexFiles.name};
    error('AssertNoRootMexArtifacts:RootMexArtifactsFound', ...
        'Repository root contains generated MEX artifacts: %s', ...
        strjoin(cellNames, ', '));
end

end
