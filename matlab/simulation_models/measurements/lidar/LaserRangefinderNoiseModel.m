function [dMeasErr] = LaserRangefinderNoiseModel(dWhiteNoiseSigma, dMeasDistance, dConstantBias)%#codegen
arguments
    dWhiteNoiseSigma    (1,1) double {isnumeric, isscalar}
    dMeasDistance       (1,1) double {isnumeric,isscalar} %#ok<INUSA>
    dConstantBias       (1,1) double {isnumeric, isscalar} = 0.0
end
%% SIGNATURE
% [dMeasErr] = LaserRangefinderNoiseModel(dMeasDistance, dWhiteNoiseSigma, dConstantBias) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function implementing measurement noise model for a Lidar rangefinder device
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dWhiteNoiseSigma    (1,1) double {isnumeric, isscalar} 
% dMeasDistance       (1,1) double {isnumeric,isscalar } %#ok<INUSA> DEVNOTE: Unused in current version
% dConstantBias       (1,1) double {isnumeric, isscalar} = 0.0
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMeasErr
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 16-01-2024    Pietro Califano     First implementation for RCS-1 simulator
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------

% Generate noise
dMeasErr = coder.nullcopy(0.0);
dMeasErr(:) = dConstantBias + dWhiteNoiseSigma * randn(1,1);

end
