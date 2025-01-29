close all
clear
clc

%% Code generation setup: evalRHS_DynOrbit function
% function o_dPosVeldt = evalRHS_DynOrbit( ...
%     i_dxState_IN, ...
%     i_dDCMmainAtt_INfromTF, ...
%     i_dMainGM, ...
%     i_dRefRmain, ...
%     i_dCoeffSRP, ...
%     i_d3rdBodiesGM, ...
%     i_dBodyEphemeris, ...
%     i_dMainCSlmCoeffCols, ...
%     i_ui32MaxSHdegree, ...
%     i_ui32StatesIdx) %#codegen
% arguments
%     i_dxState_IN
%     i_dDCMmainAtt_INfromTF
%     i_dMainGM
%     i_dRefRmain
%     i_dCoeffSRP           double = []
%     i_d3rdBodiesGM        double = []
%     i_dBodyEphemeris      double = []
%     i_dMainCSlmCoeffCols  double = []
%     i_ui32MaxSHdegree     uint32 = []
%     i_ui32StatesIdx       uint32 = []
% end

%% Coder settings
coder_config = coder.config('mex');
coder_config.TargetLang = 'C++';
coder_config.GenerateReport = true;
coder_config.LaunchReport = false;
coder_config.EnableJIT = false;
coder_config.MATLABSourceComments = true;

% IF PARALLEL REQUIRED
coder_config.EnableAutoParallelization = true;
coder_config.EnableOpenMP = true;
coder_config.OptimizeReductions = true;

%% Target function details
targetName = 'evalRHS_DynOrbit';

% numOfInputs; % ADD ASSERT to size of args_cell from specification functions

% ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
% NOTE: third option argument variable_dimensions is an array of bools, one
% for each dimension of the array

i_dxState_IN           = coder.typeof(0        , [], [1, 0]); 
i_dDCMmainAtt_INfromTF = coder.typeof(0        , [], [0, 0]); 
i_dMainGM              = coder.typeof(0        , [], [0, 0]); 
i_dRefRmain            = coder.typeof(0        , [], [0, 0]); 
i_dCoeffSRP            = coder.typeof(0        , [], [1, 1]); 
i_d3rdBodiesGM         = coder.typeof(0        , [], [1, 1]); 
i_dBodyEphemeris       = coder.typeof(0        , [], [1, 1]); 
i_dMainCSlmCoeffCols   = coder.typeof(0        , [], [1, 1]); 
i_ui32MaxSHdegree      = coder.typeof(uint32(0), [], [1, 1]); 
i_ui32StatesIdx        = coder.typeof(uint32(0), [], [1, 1]); 

args_cell{1,1} = i_dxState_IN;
args_cell{1,2} = i_dDCMmainAtt_INfromTF;
args_cell{1,3} = i_dMainGM;
args_cell{1,4} = i_dRefRmain;
args_cell{1,5} = i_dCoeffSRP;
args_cell{1,6} = i_d3rdBodiesGM;
args_cell{1,7} = i_dBodyEphemeris;
args_cell{1,8} = i_dMainCSlmCoeffCols;
args_cell{1,9} = i_ui32MaxSHdegree;
args_cell{1,10} = i_ui32StatesIdx;

% coder.getArgTypes % Function call to automatically specify input
% arguments. Note that this takes the specific sizes used in the function
% call

numOutputs = 1;
outputFileName = strcat(targetName, '_MEX');
% Defining structures 
% struct1.fieldname1 = coder.typeof(0,[3 5],1);
% struct1.fieldname2 = magic(3);
% coder.typeof(struct1); % Defines

% Defining nested structures
% S = struct('fieldname1', double(0),'fieldname2',single(0)); % Inner structure
% SuperS.x = coder.typeof(S);
% SuperS.y = single(0); 
% coder.typeof(SuperS) % Outer structure 

% IMPORTANT: in recent MATLAB versions (>2021), arguments can be speficied
% directly in function --> no need to define them outside in -args
% arguments
%     u (1,4) double
%     v (1,1) double

%% CODEGEN CALL
% Extract filename
[path2target, targetName, targetExt] = fileparts(fullfile(targetName));
% Execute code generation
codegen(strcat(targetName,'.m'), "-config", coder_config,...
    "-args", args_cell, "-nargout", numOutputs, "-o", outputFileName) 



