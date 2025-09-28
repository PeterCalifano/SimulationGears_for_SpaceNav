clear
close all
clc

%% Coder settings
objCoderConfig = coder.config('mex');
objCoderConfig.TargetLang = 'C++';
objCoderConfig.GenerateReport = true;
objCoderConfig.LaunchReport = false;
objCoderConfig.EnableJIT = false;
objCoderConfig.MATLABSourceComments = true;

objCoderConfig.EnableAutoParallelization = true;
objCoderConfig.EnableOpenMP = true;
objCoderConfig.OptimizeReductions = true;
objCoderConfig.SIMDAcceleration = 'Full';
objCoderConfig.NumberOfCpuThreads = 18;

% Common settings
ui32MaxNumPoints  = 1.2e5;
ui32MaxNumTriangs = 2e5;

% Flags for codegen calls to run
bCodegen_RayTracePointVisibility_EllipsLocalPA              = true;
bCodegen_RayTracePointVisibility_ShadowRays                 = true;
bCodegen_RayTracePointVisibility_VectorizedShadowRays       = false;

%% Target function: RayTracePointVisibility_EllipsLocalPA
if bCodegen_RayTracePointVisibility_EllipsLocalPA

    % [bAllPointsVisibilityMask, dProjectedPoints_UV] = RayTracePointVisibility_EllipsLocalPA(ui32PointsIdx, ...
    %                                                                                    dPointsPositions_TB, ...
    %                                                                                    strTargetBodyData, ...
    %                                                                                    strCameraData, ...
    %                                                                                    dSunPosition_TB, ...
    %                                                                                    strFcnOptions, ...
    %                                                                                    bDEBUG_MODE) %#codegen

    charTargetName = 'RayTracePointVisibility_EllipsLocalPA';

    % numOfInputs; % ADD ASSERT to size of args_cell from specification functions

    % ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
    % NOTE: third option argument variable_dimensions is an array of bools, one
    % for each dimension of the array
    ui32PointsIdx       = coder.typeof(uint32(0), [1, ui32MaxNumPoints], [0, 1]);
    dPointsPositions_TB = coder.typeof(0,         [3, ui32MaxNumPoints], [0, 1]);

    strCameraData.dDCM_INfromCAM = coder.typeof(0,    [3,3]);
    strCameraData.dPosition_IN   = coder.typeof(0,    [3,1]);
    strCameraData.dResX          = coder.typeof(0,    [1,1]);
    strCameraData.dResY          = coder.typeof(0,    [1,1]);
    strCameraData.dKcam          = coder.typeof(0,    [3,3]);

    strCameraData = orderfields(strCameraData);

    strShapeModel.ui32triangVertexPtr = coder.typeof(uint32(0),  [3, ui32MaxNumTriangs], [0, 1]);
    strShapeModel.dVerticesPos        = coder.typeof(0,          [3, ui32MaxNumTriangs], [0, 1]);

    strShapeModel = orderfields(strShapeModel);

    strTargetBodyData.strShapeModel    = coder.typeof(strShapeModel);
    strTargetBodyData.dDCM_INfromTB    = coder.typeof(0,    [3,3]);

    strTargetBodyData  = orderfields(strTargetBodyData);

    dSunPosition_TB                         = coder.typeof(0, [3,1]);

    strFcnOptions.dIllumAngleThr               = coder.typeof(0, [1,1]);
    strFcnOptions.dLosAngleThr                 = coder.typeof(0, [1,1]);
    strFcnOptions.bENABLE_HEUR_GEOM_FEAS_CHECK = coder.typeof(false, [1,1]);
    strFcnOptions.bTwoSidedTest                = coder.typeof(false, [1,1]);
    strFcnOptions.bPointsAreMeshVertices       = coder.typeof(false, [1,1]);
    strFcnOptions = orderfields(strFcnOptions);

    bDEBUG_MODE = coder.typeof(false, [1,1]);

    % Input arguments
    args_cell{1,1} = ui32PointsIdx;
    args_cell{1,2} = dPointsPositions_TB;
    args_cell{1,3} = strTargetBodyData;
    args_cell{1,4} = strCameraData;
    args_cell{1,5} = dSunPosition_TB;
    args_cell{1,6} = strFcnOptions;
    args_cell{1,7} = bDEBUG_MODE;

    ui32NumOutputs = 2;
    charOutputFileName = strcat(charTargetName, '_MEX');

    %  CODEGEN CALL
    % Extract filename
    [charPath2target, charTargetName, charTargetExt] = fileparts(fullfile(charTargetName)); %#ok<ASGLU>
    % Execute code generation
    codegen(strcat(charTargetName,'.m'), "-config", objCoderConfig,...
        "-args", args_cell, "-nargout", ui32NumOutputs, "-o", charOutputFileName)

end


%% Target function: RayTracePointVisibility_ShadowRays
if bCodegen_RayTracePointVisibility_ShadowRays

    % [bAllPointsVisibilityMask, dProjectedPoints_UV] = RayTracePointVisibility_ShadowRays(ui32PointsIdx, ...
    %                                                                                                dPointsPositions_TB, ...
    %                                                                                                strTargetBodyData, ...
    %                                                                                                strCameraData, ...
    %                                                                                                dSunPosition_TB, ...
    %                                                                                                bDEBUG_MODE, ...
    %                                                                                                bTwoSidedTest, ...
    %                                                                                                bPointsAreMeshVertices, ...
    %                                                                                                bSkipIlluminationCheck) %#codegen
    charTargetName = 'RayTracePointVisibility_ShadowRays';

    % numOfInputs; % ADD ASSERT to size of args_cell from specification functions

    % ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
    ui32PointsIdx       = coder.typeof(uint32(0), [1, ui32MaxNumPoints], [0, 1]);
    dPointsPositions_TB = coder.typeof(0,         [3, ui32MaxNumPoints], [0, 1]);

    strCameraData.dDCM_INfromCAM = coder.typeof(0,    [3,3]);
    strCameraData.dPosition_IN   = coder.typeof(0,    [3,1]);
    strCameraData.dResX          = coder.typeof(0,    [1,1]);
    strCameraData.dResY          = coder.typeof(0,    [1,1]);
    strCameraData.dKcam          = coder.typeof(0,    [3,3]);

    strCameraData = orderfields(strCameraData);


    strShapeModel.ui32triangVertexPtr = coder.typeof(uint32(0),  [3, ui32MaxNumTriangs], [0, 1]);
    strShapeModel.dVerticesPos        = coder.typeof(0,         [3, ui32MaxNumTriangs], [0, 1]);

    strShapeModel = orderfields(strShapeModel);

    strTargetBodyData.strShapeModel    = coder.typeof(strShapeModel);
    strTargetBodyData.dDCM_INfromTB    = coder.typeof(0,    [3,3]);

    strTargetBodyData  = orderfields(strTargetBodyData);

    dSunPosition_TB         = coder.typeof(0, [3,1]);
    bDEBUG_MODE             = coder.typeof(false, [1,1]);
    bTwoSidedTest           = coder.typeof(false, [1,1]);
    bPointsAreMeshVertices  = coder.typeof(false, [1,1]);
    bSkipIlluminationCheck  = coder.typeof(false, [1,1]);

    % Input arguments
    args_cell{1,1} = ui32PointsIdx;
    args_cell{1,2} = dPointsPositions_TB;
    args_cell{1,3} = strTargetBodyData;
    args_cell{1,4} = strCameraData;
    args_cell{1,5} = dSunPosition_TB;
    args_cell{1,6} = bDEBUG_MODE;
    args_cell{1,7} = bTwoSidedTest;
    args_cell{1,8} = bPointsAreMeshVertices;
    args_cell{1,9} = bSkipIlluminationCheck;

    ui32NumOutputs = 2;
    charOutputFileName = strcat(charTargetName, '_MEX');

    %  CODEGEN CALL
    % Extract filename
    [charPath2target, charTargetName, charTargetExt] = fileparts(fullfile(charTargetName));
    % Execute code generation
    codegen(strcat(charTargetName,'.m'), "-config", objCoderConfig,...
        "-args", args_cell, "-nargout", ui32NumOutputs, "-o", charOutputFileName)
end

return
%%% DEPRECATED
if bCodegen_RayTracePointVisibility_VectorizedShadowRays && 0 %#ok<UNRCH>
    % DEVNOTE: disabled. This function is not up to date.

    % [bAllPointsVisibilityMask_ParallelRTwithShadowRays, dProjectedPoints_UV] = ParallelRayTracePointVisibility_ShadowRays(uint32(ui32pointsIDs), ...
    %                                                                                                                         dPointsPositionsGT_TB, ...
    %                                                                                                                         strTargetBodyData, ...
    %                                                                                                                         strCameraData, ...
    %                                                                                                                         dSunPosition_TB, ...
    %                                                                                                                         bDEBUG_MODE, ...
    %                                                                                                                         bTwoSidedTest);
    charTargetName = 'RayTracePointVisibility_VectorizedShadowRays';

    % numOfInputs; % ADD ASSERT to size of args_cell from specification functions

    % ENTRY-POINT FUNCTION ARGUMENTS DEFINITION
    % NOTE: third option argument variable_dimensions is an array of bools, one
    % for each dimension of the array
    ui32PointsIdx       = coder.typeof(uint32(0), [1, ui32MaxNumPoints], [0, 1]);
    dPointsPositions_TB = coder.typeof(0,         [3, ui32MaxNumPoints], [0, 1]);

    strCameraData.dDCM_INfromCAM = coder.typeof(0,    [3,3]);
    strCameraData.dPosition_IN   = coder.typeof(0,    [3,1]);
    strCameraData.dResX          = coder.typeof(0,    [1,1]);
    strCameraData.dResY          = coder.typeof(0,    [1,1]);
    strCameraData.dKcam          = coder.typeof(0,    [3,3]);

    strCameraData = orderfields(strCameraData);


    strShapeModel.ui32triangVertexPtr = coder.typeof(int32(0),  [3, ui32MaxNumTriangs], [0, 1]);
    strShapeModel.dVerticesPos        = coder.typeof(0,         [3, ui32MaxNumTriangs], [0, 1]);

    strShapeModel = orderfields(strShapeModel);

    strTargetBodyData.strShapeModel    = coder.typeof(strShapeModel);
    strTargetBodyData.dDCM_INfromTB    = coder.typeof(0,    [3,3]);

    strTargetBodyData  = orderfields(strTargetBodyData);

    dSunPosition_TB                         = coder.typeof(0, [3,1]);
    bDEBUG_MODE     = coder.typeof(false, [1,1]);
    bTwoSidedTest   = coder.typeof(false, [1,1]);

    % Function args
    % ui32PointsIdx       (1,:) uint32
    % dPointsPositions_TB (3,:) double
    % strTargetBodyData   {isstruct}
    % strCameraData       {isstruct}
    % dSunDir_TB          (3,1) double
    % bDEBUG_MODE         (1,1) logical {islogical} = false
    % bTwoSidedTest         (1,1) logical {islogical} = false

    args_cell{1,1} = ui32PointsIdx;
    args_cell{1,2} = dPointsPositions_TB;
    args_cell{1,3} = strTargetBodyData;
    args_cell{1,4} = strCameraData;
    args_cell{1,5} = dSunPosition_TB;
    args_cell{1,6} = bDEBUG_MODE;
    args_cell{1,7} = bTwoSidedTest;

    % coder.getArgTypes % Function call to automatically specify input
    % arguments. Note that this takes the specific sizes used in the function
    % call

    ui32NumOutputs = 2;
    charOutputFileName = strcat(charTargetName, '_MEX');

    %  CODEGEN CALL
    % Extract filename
    [charPath2target, charTargetName, charTargetExt] = fileparts(fullfile(charTargetName));
    % Execute code generation
    codegen(strcat(charTargetName,'.m'), "-config", objCoderConfig,...
        "-args", args_cell, "-nargout", ui32NumOutputs, "-o", charOutputFileName)
end
