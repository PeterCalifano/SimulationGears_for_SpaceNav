clear
close all
clc

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

% Common settings
ui32MaxNumTriangs = 6e4;


bCodegen_RayTriangleIntersection_MollerTrumbore               = true;
bCodegen_RayTriangleVectorizedIntersect_MollerTrumbore        = true;
bCodegen_RayTriangleVectIntersectWithReject_MollerTrumbore    = true;

%% Target function: RayTracePointVisibility_EllipsLocalPA
if bCodegen_RayTriangleIntersection_MollerTrumbore
    targetName = 'RayTriangleIntersection_MollerTrumbore';

    % ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
    % NOTE: third option argument variable_dimensions is an array of bools, one
    % for each dimension of the array

    % [bIntersectionFlag, dUbarycenCoord, dVbarycenCoord, ...
    %           dtRangeToIntersection, dIntersectionPoint] =  RayTriangleIntersection_MollerTrumbore( ...
    %                                                                 dRayOrigin, ...
    %                                                                 dRayDirection, ...
    %                                                                 dTriangVert0, ...
    %                                                                 dTriangVert1, ...
    %                                                                 dTriangVert2, ...
    %                                                                 bTwoSidedTest, ...
    %                                                                 bCheckOcclusionOnly) %#codegen

    % Function args
    dRayOrigin          = coder.typeof(0,  [3, 1], [0, 0]);
    dRayDirection       = coder.typeof(0,  [3, 1], [0, 0]);
    dTriangVert0        = coder.typeof(0,  [3, 1], [0, 0]);
    dTriangVert1        = coder.typeof(0,  [3, 1], [0, 0]);
    dTriangVert2        = coder.typeof(0,  [3, 1], [0, 0]);
    bTwoSidedTest       = coder.typeof(false, [1, 1], [0, 0]);
    bCheckOcclusionOnly = coder.typeof(false, [1, 1], [0, 0]);

    args_cell{1,1} = dRayOrigin;
    args_cell{1,2} = dRayDirection;
    args_cell{1,3} = dTriangVert0;
    args_cell{1,4} = dTriangVert1;
    args_cell{1,5} = dTriangVert2;
    args_cell{1,6} = bTwoSidedTest;
    args_cell{1,7} = bCheckOcclusionOnly;

    % coder.getArgTypes % Function call to automatically specify input
    % arguments. Note that this takes the specific sizes used in the function
    % call

    numOutputs = 5;
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
  

    %  CODEGEN CALL
    % Extract filename
    [path2target, targetName, ~] = fileparts(fullfile(targetName)); %#ok<*ASGLU>
    % Execute code generation
    codegen(strcat(targetName,'.m'), "-config", coder_config,...
        "-args", args_cell, "-nargout", numOutputs, "-o", outputFileName)

end


%% Target function: RayTracePointVisibility_ShadowRays
if bCodegen_RayTriangleVectorizedIntersect_MollerTrumbore

    % [bAllPointsVisibilityMask_ParallelRTwithShadowRays, dProjectedPoints_UV] = ParallelRayTracePointVisibility_ShadowRays(uint32(ui32pointsIDs), ...
    %                                                                                                                         dPointsPositionsGT_TB, ...
    %                                                                                                                         strTargetBodyData, ...
    %                                                                                                                         strCameraData, ...
    %                                                                                                                         dSunPosition_TB, ...
    %                                                                                                                         bDEBUG_MODE, ...
    %                                                                                                                         bTwoSidedTest);
    targetName = 'RayTriangleVectorizedIntersect_MollerTrumbore';

    % numOfInputs; % ADD ASSERT to size of args_cell from specification functions

    % ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
    % NOTE: third option argument variable_dimensions is an array of bools, one
    % for each dimension of the array

    % [bIsIntersected, dUbarycenCoord, dVbarycenCoord, ...
    %     dRangeToIntersection, dIntersectionPoint] = RayTriangleVectorizedIntersect_MollerTrumbore(dRayOrigin, ...
    %                                                                                              dRayDirection, ...
    %                                                                                              dAllTriangVert0, ...
    %                                                                                              dAllTriangVert1, ...
    %                                                                                              dAllTriangVert2, ...
    %                                                                                              ui32NumOfTriang, ...
    %                                                                                              bTwoSidedTest)%#codegen

    % Function args
    dRayOrigin      = coder.typeof(0,  [3, 1], [0, 0]);
    dRayDirection   = coder.typeof(0,  [3, 1], [0, 0]);
    dAllTriangVert0 = coder.typeof(0,  [3, ui32MaxNumTriangs], [0, 1]);
    dAllTriangVert1 = coder.typeof(0,  [3, ui32MaxNumTriangs], [0, 1]);
    dAllTriangVert2 = coder.typeof(0,  [3, ui32MaxNumTriangs], [0, 1]);
    ui32NumOfTriang = coder.typeof(uint32(0),  [1, 1], [0, 0]);
    bTwoSidedTest   = coder.typeof(false, [1, 1], [0, 0]);

    args_cell{1,1}  = dRayOrigin;
    args_cell{1,2}  = dRayDirection;
    args_cell{1,3}  = dAllTriangVert0;
    args_cell{1,4}  = dAllTriangVert1;
    args_cell{1,5}  = dAllTriangVert2;
    args_cell{1,6}  = ui32NumOfTriang;
    args_cell{1,7}  = bTwoSidedTest;

    % coder.getArgTypes % Function call to automatically specify input
    % arguments. Note that this takes the specific sizes used in the function
    % call

    numOutputs = 5;
    outputFileName = strcat(targetName, '_MEX');

    %  CODEGEN CALL
    % Extract filename
    [path2target, targetName, targetExt] = fileparts(fullfile(targetName));
    % Execute code generation
    codegen(strcat(targetName,'.m'), "-config", coder_config,...
        "-args", args_cell, "-nargout", numOutputs, "-o", outputFileName)
end



%% Target function: RayTriangleVectorizedIntersectWithReject_MollerTrumbore

if bCodegen_RayTriangleVectIntersectWithReject_MollerTrumbore

    % [bAllPointsVisibilityMask_ParallelRTwithShadowRays, dProjectedPoints_UV] = ParallelRayTracePointVisibility_ShadowRays(uint32(ui32pointsIDs), ...
    %                                                                                                                         dPointsPositionsGT_TB, ...
    %                                                                                                                         strTargetBodyData, ...
    %                                                                                                                         strCameraData, ...
    %                                                                                                                         dSunPosition_TB, ...
    %                                                                                                                         bDEBUG_MODE, ...
    %                                                                                                                         bTwoSidedTest);
    targetName = 'RayTriangleVectorizedIntersectWithReject_MollerTrumbore';

    % numOfInputs; % ADD ASSERT to size of args_cell from specification functions

    % ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
    % NOTE: third option argument variable_dimensions is an array of bools, one
    % for each dimension of the array

    %
    % [bIsIntersected, dUbarycenCoord, dVbarycenCoord, ...
    %     dRangeToIntersection, dIntersectionPoint] = RayTriangleVectorizedIntersectWithReject_MollerTrumbore(dRayOrigin, ...
    %                                                                                              dRayDirection, ...
    %                                                                                              dAllTriangVert0, ...
    %                                                                                              dAllTriangVert1, ...
    %                                                                                              dAllTriangVert2, ...
    %                                                                                              ui32NumOfTriang, ...
    %                                                                                              bTwoSidedTest)%#codegen


    % Function args
    dRayOrigin      = coder.typeof(0,  [3, 1], [0, 0]);
    dRayDirection   = coder.typeof(0,  [3, 1], [0, 0]);
    dAllTriangVert0 = coder.typeof(0,  [3, ui32MaxNumTriangs], [0, 1]);
    dAllTriangVert1 = coder.typeof(0,  [3, ui32MaxNumTriangs], [0, 1]);
    dAllTriangVert2 = coder.typeof(0,  [3, ui32MaxNumTriangs], [0, 1]);
    ui32NumOfTriang = coder.typeof(uint32(0),  [1, 1], [0, 0]);
    bTwoSidedTest   = coder.typeof(false, [1, 1], [0, 0]);

    args_cell{1,1}  = dRayOrigin;
    args_cell{1,2}  = dRayDirection;
    args_cell{1,3}  = dAllTriangVert0;
    args_cell{1,4}  = dAllTriangVert1;
    args_cell{1,5}  = dAllTriangVert2;
    args_cell{1,6}  = ui32NumOfTriang;
    args_cell{1,7}  = bTwoSidedTest;

    % coder.getArgTypes % Function call to automatically specify input
    % arguments. Note that this takes the specific sizes used in the function
    % call

    numOutputs = 5;
    outputFileName = strcat(targetName, '_MEX');

    %  CODEGEN CALL
    % Extract filename
    [path2target, targetName, targetExt] = fileparts(fullfile(targetName));
    % Execute code generation
    codegen(strcat(targetName,'.m'), "-config", coder_config,...
        "-args", args_cell, "-nargout", numOutputs, "-o", outputFileName)
end


%% Target function: ParallelRayTracePointVisibility_ShadowRays
% ACHTUNG: parallel.pool.Constant does not support code generation?!
% targetName = 'ParallelRayTracePointVisibility_ShadowRays';
% 
% % numOfInputs; % ADD ASSERT to size of args_cell from specification functions
% 
% % ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
% % NOTE: third option argument variable_dimensions is an array of bools, one
% % for each dimension of the array
% ui32PointsIdx       = coder.typeof(uint32(0), [1, ui32MaxNumPoints], [0, 1]);
% dPointsPositions_TB = coder.typeof(0,         [3, ui32MaxNumPoints], [0, 1]);
% 
% strCameraData.dDCM_INfromCAM = coder.typeof(0,    [3,3]);
% strCameraData.dPosition_IN   = coder.typeof(0,    [3,1]);
% strCameraData.dResX          = coder.typeof(0,    [1,1]);
% strCameraData.dResY          = coder.typeof(0,    [1,1]);
% strCameraData.dKcam          = coder.typeof(0,    [3,3]);
% 
% strCameraData = orderfields(strCameraData);
% 
% 
% strShapeModel.ui32triangVertexPtr = coder.typeof(int32(0),  [3, ui32MaxNumTriangs], [0, 1]);
% strShapeModel.dVerticesPos        = coder.typeof(0,         [3, ui32MaxNumTriangs], [0, 1]);
% 
% strShapeModel = orderfields(strShapeModel);
% 
% strTargetBodyData.strShapeModel    = coder.typeof(strShapeModel);
% strTargetBodyData.dDCM_INfromTB    = coder.typeof(0,    [3,3]);
% 
% strTargetBodyData  = orderfields(strTargetBodyData);
% 
% dSunDir_TB      = coder.typeof(0, [3,1]);
% bDEBUG_MODE     = coder.typeof(false, [1,1]);
% bTwoSidedTest   = coder.typeof(false, [1,1]);
% 
% % Function args
% % ui32PointsIdx       (1,:) uint32
% % dPointsPositions_TB (3,:) double
% % strTargetBodyData   {isstruct}
% % strCameraData       {isstruct}
% % dSunDir_TB          (3,1) double
% % bDEBUG_MODE         (1,1) false {islogical} = false
% % bTwoSidedTest         (1,1) false {islogical} = false
% 
% args_cell{1,1} = ui32PointsIdx;
% args_cell{1,2} = dPointsPositions_TB;
% args_cell{1,3} = strTargetBodyData;
% args_cell{1,4} = strCameraData;
% args_cell{1,5} = dSunDir_TB;
% args_cell{1,6} = bDEBUG_MODE;
% args_cell{1,7} = bTwoSidedTest;
% 
% % coder.getArgTypes % Function call to automatically specify input
% % arguments. Note that this takes the specific sizes used in the function
% % call
% 
% numOutputs = 2;
% outputFileName = strcat(targetName, '_MEX');
% 
% %  CODEGEN CALL
% % Extract filename
% [path2target, targetName, targetExt] = fileparts(fullfile(targetName));
% % Execute code generation
% codegen(strcat(targetName,'.m'), "-config", coder_config,...
%     "-args", args_cell, "-nargout", numOutputs, "-o", outputFileName)
