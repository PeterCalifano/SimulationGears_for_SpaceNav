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
coder_config.OptimizeReductions = true;

%% Target function details
targetName = 'FastRayTracePointVisibility';

% numOfInputs; % ADD ASSERT to size of args_cell from specification functions

% ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
% NOTE: third option argument variable_dimensions is an array of bools, one
% for each dimension of the array
ui32PointsIdx       = coder.typeof(uint32(0), [1, Inf], [0, 1]);
dPointsPositions_TB = coder.typeof(0,         [3, Inf], [0, 1]);

strCameraData.dDCM_INfromCAM = coder.typeof(0,    [3,3]);
strCameraData.dPosition_IN   = coder.typeof(0,    [3,1]);
strCameraData.dResX          = coder.typeof(0,    [1,1]);
strCameraData.dResY          = coder.typeof(0,    [1,1]);
strCameraData.dKcam          = coder.typeof(0,    [3,3]);

strCameraData = orderfields(strCameraData);


strShapeModel.ui32triangVertexPtr = coder.typeof(int32(0),  [3, Inf], [0, 1]);
strShapeModel.dVerticesPos        = coder.typeof(0,         [3, Inf], [0, 1]);

strShapeModel = orderfields(strShapeModel);

strTargetBodyData.strShapeModel    = coder.typeof(strShapeModel);
strTargetBodyData.dDCM_INfromTB    = coder.typeof(0,    [3,3]);

strTargetBodyData  = orderfields(strTargetBodyData);

dSunDir_TB                         = coder.typeof(0, [3,1]);

strFcnOptions.dIllumAngleThr               = coder.typeof(0, [1,1]);
strFcnOptions.dLosAngleThr                 = coder.typeof(0, [1,1]);
strFcnOptions.bENABLE_HEUR_GEOM_FEAS_CHECK = coder.typeof(false, [1,1]);

strFcnOptions = orderfields(strFcnOptions);

bDEBUG_MODE = coder.typeof(false, [1,1]);
% Function args
% ui32PointsIdx       (1,:) uint32
% dPointsPositions_TB (3,:) double
% strTargetBodyData   {isstruct}
% strCameraData       {isstruct}
% dSunDir_TB          (3,1) double
% strFcnOptions       {isstruct}
% bDEBUG_MODE         (1,1) logical {islogical} = false

args_cell{1,1} = ui32PointsIdx;
args_cell{1,2} = dPointsPositions_TB;
args_cell{1,3} = strTargetBodyData;
args_cell{1,4} = strCameraData;
args_cell{1,5} = dSunDir_TB;
args_cell{1,6} = strFcnOptions;
args_cell{1,7} = bDEBUG_MODE;

% coder.getArgTypes % Function call to automatically specify input
% arguments. Note that this takes the specific sizes used in the function
% call

numOutputs = 2;
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



