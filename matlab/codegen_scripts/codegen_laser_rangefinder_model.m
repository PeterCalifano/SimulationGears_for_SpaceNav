clear
close all
clc

charThisScriptFolder = fileparts(mfilename('fullpath'));
cd(charThisScriptFolder);
addpath(genpath(".."));

%% Coder settings
coder_config = coder.config('mex');
coder_config.TargetLang = 'C++';
coder_config.GenerateReport = true;
coder_config.LaunchReport = false;
coder_config.EnableJIT = false;
coder_config.MATLABSourceComments = true;

coder_config.EnableAutoParallelization = true;
coder_config.EnableOpenMP = true;
coder_config.OptimizeReductions = true;
coder_config.SIMDAcceleration = 'Full';

% Common settings
ui32MaxNumTriangs  = 2e6;
ui32MaxNumVertices = 5e6;

%% Target function: LaserRangefinderModel

% [dMeasDistance, bInsersectionFlag, bValidityFlag, dIntersectionPoint, dMeasErr] = LaserRangefinderModel( ...
%                                                                                              strTargetModelData, ...
%                                                                                              dBeamDirection_TB, ...
%                                                                                              dSensorOrigin_TB, ...
%                                                                                              dMeasWhiteNoiseSigma, ...
%                                                                                              dConstantBias, ...
%                                                                                              dTargetPosition_TB, ...
%                                                                                              dMeasValidInterval, ...
%                                                                                              bEnableNoiseModels, ...
%                                                                                              bEnableHeuristicPruning, ...
%                                                                                              bEnableValidityChecks) %#codegen

charTargetName = 'LaserRangefinderModel';

% ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
strTargetModelData.i32triangVertexPtrs = coder.typeof(int32(0), [3, ui32MaxNumTriangs], [0, 1]);
strTargetModelData.dVerticesPositions  = coder.typeof(0,        [3, ui32MaxNumVertices], [0, 1]);
strTargetModelData = orderfields(strTargetModelData);

dBeamDirection_TB       = coder.typeof(0, [3,1]);
dSensorOrigin_TB        = coder.typeof(0, [3,1]);
dMeasWhiteNoiseSigma    = coder.typeof(0, [1,1]);
dConstantBias           = coder.typeof(0, [1,1]);
dTargetPosition_TB      = coder.typeof(0, [3,1]);
dMeasValidInterval      = coder.typeof(0, [2,1]);
bEnableNoiseModels      = coder.typeof(false, [1,1]);
bEnableHeuristicPruning = coder.typeof(false, [1,1]);
bEnableValidityChecks   = coder.typeof(false, [1,1]);

% Input arguments
args_cell{1,1}  = strTargetModelData;
args_cell{1,2}  = dBeamDirection_TB;
args_cell{1,3}  = dSensorOrigin_TB;
args_cell{1,4}  = dMeasWhiteNoiseSigma;
args_cell{1,5}  = dConstantBias;
args_cell{1,6}  = dTargetPosition_TB;
args_cell{1,7}  = dMeasValidInterval;
args_cell{1,8}  = bEnableNoiseModels;
args_cell{1,9}  = bEnableHeuristicPruning;
args_cell{1,10} = bEnableValidityChecks;

ui32NumOutputs = 5;
charOutputFileName = strcat(charTargetName, '_MEX');

% CODEGEN CALL
[charPath2target, charTargetName, charTargetExt] = fileparts(fullfile(charTargetName));
codegen(strcat(charTargetName,'.m'), "-config", coder_config,...
    "-args", args_cell, "-nargout", ui32NumOutputs, "-o", charOutputFileName)
