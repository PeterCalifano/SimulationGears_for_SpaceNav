close all
clear
clc


%% TEST: CheckLMvisibility_rayTrace (LEGACY)
% Created by PeterC 21-04-2024
% Function declaration:
% [o_dIsLMvisibleMask] = CheckLMvisibility_rayTrace(i_dLMposTable_TB, ...
%     i_strTargetBody, i_strCamera, i_dSunDir_TB, i_strFcnOptions)

i_dLMposTable_TB = load("mappedLMtables/LandmarksMapDidymos2023_Nlm1000_19_Apr_2024_18-50").o_dLMposMap;

kernelname = char(fullfile("data", "didymos_g_09309mm_spc_0000n00000_v003.bds"));
cspice_furnsh( kernelname );     
[filetype, sourcefile, kernelhandle] = cspice_kinfo(kernelname);
dladsc = cspice_dlabfs(kernelhandle); % What does this function do?

% Get number of vertices and triangles
[nVertices, nTriangles] = cspice_dskz02(kernelhandle, dladsc);
% Get triangles from SPICE (p: plates)
trianglesVertices = cspice_dskp02(kernelhandle, dladsc, 1, nTriangles);

% Get vertices from SPICE (v: vertices)
modelVertices = 1000*cspice_dskv02(kernelhandle, dladsc, 1, nVertices);


strShapeModel.i32triangVertexPtr = trianglesVertices;
strShapeModel.dVerticesPos =  modelVertices;

idLMarray = 1:size(i_dLMposTable_TB, 2);
initializedLMsMask = false(length(idLMarray), 1);
isLMstored = false(length(idLMarray), 1);
tmpObsBuffer = zeros(2, length(idLMarray));

% OPTIONS
i_strFcnOptions.dIllumAngleThr = deg2rad(80); % [rad] Determines max angle between Sun direction and LM direction
% Determines max angle between -los and LM direction in TF. Angle to assign is between +Los and the
i_strFcnOptions.dLosAngleThr = deg2rad(80); % [rad]

% TARGET BODY
strShapeModel = orderfields(strShapeModel);
i_strTargetBody.strShapeModel = strShapeModel;
i_strTargetBody.dTargetPos_IN = [0;0;0]; % Target Fixed frame is at origin

i_strTargetBody.dQuat_fromTBtoIN = [0; 0; 0; 1]; % TEMPORARY: replace with attitude ephemerides
i_strTargetBody.bIS_JPL_QUAT = true;

% i_strCamera.sensor_size_x = 2048;  % [pxl]
% i_strCamera.sensor_size_y = 1536;  % [pxl]
i_strCamera.dFovX         = deg2rad(21); % [rad]
i_strCamera.dFovY         = deg2rad(16); % [rad]
% i_strCamera.dKcam         = [ 7286.14, 0, 1024;
%                               0, 4143.76, 768;
%                               0, 0, 1];

i_strCamera.dQuat_fromCAMtoIN = [0; 0.7071; 0; 0.7071];

i_strCamera.bIS_JPL_QUAT = true;
i_strCamera.dCAMpos_IN = [5000; 0; 0];

i_dSunDir_TB = [1; 0; 0];


% For mex compatibility, use orderfields
i_strCamera     = orderfields(i_strCamera);
i_strFcnOptions = orderfields(i_strFcnOptions);
i_strTargetBody = orderfields(i_strTargetBody);

% MATLAB VERSION
tic
[o_dIsLMvisibleMask] = CheckLMvisibility_rayTrace(i_dLMposTable_TB, ...
    i_strTargetBody, i_strCamera, i_dSunDir_TB, i_strFcnOptions);
toc

%% EQUIVALENCE TEST
% MEX VERSION
tic
[o_dIsLMvisibleMask_mex] = CheckLMvisibility_rayTrace_MEX(i_dLMposTable_TB, ...
    i_strTargetBody, i_strCamera, i_dSunDir_TB, i_strFcnOptions);
toc

outputDiff = o_dIsLMvisibleMask ~= o_dIsLMvisibleMask_mex;
sum(abs(outputDiff))
% assert(any(abs(outputDiff) > 5*eps), 'Test failed: errors too large to be due to finite arithmetic.')



