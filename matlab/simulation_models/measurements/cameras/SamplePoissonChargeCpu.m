function dCounts = SamplePoissonChargeCpu(dExpectedElectrons) %#codegen
%% SIGNATURE
% dCounts = SamplePoissonChargeCpu(dExpectedElectrons)
% -------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Provide the hosted CPU code-generation entry for Poisson charge sampling. Keep the
% statistical operation identical to poissrnd; generated random draws need not equal
% interpreted draws. The caller owns initialization of MATLAB's random stream.
% -------------------------------------------------------------------------------------------------
%% INPUT
% dExpectedElectrons   Finite nonnegative expected charge, in electrons.
% -------------------------------------------------------------------------------------------------
%% OUTPUT
% dCounts              Double charge counts with the input array shape.
% -------------------------------------------------------------------------------------------------
%% CHANGELOG
% 19-10-2026  Pietro Califano     Add the maintained CPU MEX fallback entry.
% 11-11-2026  Pietro Califano, Claude Opus 5.5  Readability pass.
% -------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Statistics and Machine Learning Toolbox; MATLAB Coder for MEX generation.
% -------------------------------------------------------------------------------------------------

%% Function code

arguments (Input)
    dExpectedElectrons (:,:) double {mustBeReal, mustBeFinite, mustBeNonnegative}
end
arguments (Output)
    dCounts (:,:) double
end

% Draw one Poisson count per pixel
dCounts = poissrnd(dExpectedElectrons);
end
