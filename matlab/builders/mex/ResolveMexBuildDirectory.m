function charBuildDir = ResolveMexBuildDirectory(charBuildDir, varargin)
%% PROTOTYPE
% charBuildDir = ResolveMexBuildDirectory(charBuildDir, varargin)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Resolves a MEX build directory. Explicit build directories are preserved.
% Empty build directories default to matlab/mex/<varargin source layout>.
% -------------------------------------------------------------------------------------------------------------

if nargin < 1 || strlength(string(charBuildDir)) == 0
    charResolverDir = fileparts(mfilename('fullpath'));
    charMatlabRoot = fileparts(fileparts(charResolverDir));
    charBuildDir = fullfile(charMatlabRoot, 'mex', varargin{:});
else
    charBuildDir = char(string(charBuildDir));
end

if ~exist(charBuildDir, 'dir')
    mkdir(charBuildDir);
end

end
