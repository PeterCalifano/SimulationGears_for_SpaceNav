close all
clear
clc

%% CODEGEN SCRIPT: GyrosModel
% Created by PeterC, 20-06-2024, PhD course Lab2

targetFcnName = "GyrosModel";

% Input definition
i_dTrueAngRate          = coder.typeof(0, [3,1], [false, false]); % (3,1) double
i_dScaleFactor          = coder.typeof(0, [1,1], [false, false]); % (1,1) double
i_dOrthogErrMatrix      = coder.typeof(0, [3,3], [false, false]); % (3,3) double
i_dConfigMat_IMUfromSCB = coder.typeof(0, [3,3], [false, false]); % (3,3) double
i_dSigmaRRW             = coder.typeof(0, [1,1], [false, false]); % (1,1) double
i_dSigmaARW             = coder.typeof(0, [1,1], [false, false]); % (1,1) double
i_dPrevBiasValue        = coder.typeof(0, [3,1], [false, false]); % (3,1) double

args_cell{1} = i_dTrueAngRate          ;
args_cell{2} = i_dScaleFactor          ;
args_cell{3} = i_dOrthogErrMatrix      ;
args_cell{4} = i_dConfigMat_IMUfromSCB ;
args_cell{5} = i_dSigmaRRW             ;
args_cell{6} = i_dSigmaARW             ;
args_cell{7} = i_dPrevBiasValue        ;


makeCodegen(targetFcnName, args_cell, 'lib');
