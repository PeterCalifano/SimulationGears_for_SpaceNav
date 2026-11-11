function charMexPath = BuildMexTargets_CameraPoisson(charOutputPath)
%% SIGNATURE
% charMexPath = BuildMexTargets_CameraPoisson(charOutputPath)
% -------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Build the independent CPU Poisson MEX fallback in the owned MATLAB MEX directory,
% or an explicit output directory. Variable-size two-dimensional inputs permit reuse
% across camera resolutions; the simulation's resource guard owns memory admission.
% -------------------------------------------------------------------------------------------------
%% INPUT
% charOutputPath    Optional output directory; empty selects this library's matlab/mex.
% -------------------------------------------------------------------------------------------------
%% OUTPUT
% charMexPath       Full path of DrawPoissonCountsCpu_mex for this MATLAB platform.
% -------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-10-2026  Pietro Califano     Add the maintained hosted CPU fallback builder.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% MATLAB Coder, SamplePoissonChargeCpu, configured supported C++ MEX compiler.
% -------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    charOutputPath (1,:) char = ''
end
arguments (Output)
    charMexPath (1,:) char
end

% Default to the matlab/mex folder of this library
charMatlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if isempty(charOutputPath)
    charOutputPath = fullfile(charMatlabRoot, 'mex');
end
if ~isfolder(charOutputPath)
    mkdir(charOutputPath);
end

% Generate a C++ MEX that accepts any two-dimensional double array
charBuildPath = fullfile(charOutputPath, 'codegen', 'camera_poisson');
objConfig = coder.config('mex');
objConfig.TargetLang = 'C++';
objConfig.GenerateReport = false;
charTargetPath = fullfile(charOutputPath, 'DrawPoissonCountsCpu_mex');
codegen('-config', objConfig, ...
        'SamplePoissonChargeCpu', ...
        '-args', {coder.typeof(0, [Inf, Inf], [true, true])}, ...
        '-d', charBuildPath, ...
        '-o', charTargetPath);
charMexPath = [charTargetPath, '.', mexext];
end
