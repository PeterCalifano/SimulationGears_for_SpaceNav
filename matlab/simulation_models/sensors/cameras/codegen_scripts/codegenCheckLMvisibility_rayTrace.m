% clear
% close all
% clc

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

%% Target function details
targetName = 'CheckLMvisibility_rayTrace';

% numOfInputs; % ADD ASSERT to size of args_cell from specification functions

% ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
% NOTE: third option argument variable_dimensions is an array of bools, one
% for each dimension of the array


i_strCamera_MEX.dFovY                = coder.typeof(0, [1,1]);
i_strCamera_MEX.dFovX                = coder.typeof(0, [1,1]);
i_strCamera_MEX.dCAMpos_IN           = coder.typeof(0, [3,1]);
i_strCamera_MEX.bIS_JPL_QUAT         = coder.typeof(true, [1,1]);
i_strCamera_MEX.dQuat_INfromCAM      = coder.typeof(0, [4,1]);
i_strCamera_MEX = orderfields(i_strCamera_MEX);


strShapeModel_MEX.ui32triangVertexPtr = coder.typeof(int32(0),  [3, Inf], [0, 1]);
strShapeModel_MEX.dVerticesPos        = coder.typeof(0, [3, Inf], [0, 1]);

strShapeModel_MEX = orderfields(strShapeModel_MEX);

i_strTargetBody_MEX.dQuat_INfromTB   = coder.typeof(0, [4,1]);
i_strTargetBody_MEX.strShapeModel    = coder.typeof(strShapeModel_MEX);
i_strTargetBody_MEX.dTargetPos_IN    = coder.typeof(0, [3,1]);
i_strTargetBody_MEX.bIS_JPL_QUAT     = coder.typeof(true, [1,1]);
i_strTargetBody_MEX  = orderfields(i_strTargetBody_MEX);

i_dSunDir_TB_MEX                     = coder.typeof(0, [3,1]);

i_strFcnOptions_MEX.dIllumAngleThr   = coder.typeof(0, [1,1]);
i_strFcnOptions_MEX.dLosAngleThr     = coder.typeof(0, [1,1]);
i_strFcnOptions_MEX = orderfields(i_strFcnOptions_MEX);

i_dLMposTable_TB_MEX                 = coder.typeof(0, [4,Inf], [0, 1]);
% MATLAB VERSION
% [o_dIsLMvisibleMask] = CheckLMvisibility_rayTrace(i_dLMposTable_TB, ...
%     i_strTargetBody, i_strCamera, i_dSunDir_TB, i_strFcnOptions);

% coder.typeof(example_value, size_vector, variable_dims);

args_cell{1,1} = i_dLMposTable_TB_MEX;
args_cell{1,2} = i_strTargetBody_MEX;
args_cell{1,3} = i_strCamera_MEX;
args_cell{1,4} = i_dSunDir_TB_MEX;
args_cell{1,5} = i_strFcnOptions_MEX;

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



