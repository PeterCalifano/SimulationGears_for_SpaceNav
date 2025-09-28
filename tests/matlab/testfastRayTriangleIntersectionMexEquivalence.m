close all
clear
clc

%% TEST: fastRayTriangleIntersection
% Created by PeterC 01-05-2024

kernelname = char(fullfile("../data", "didymos_g_09309mm_spc_0000n00000_v003.bds"));
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
Nlandmarks = 1000;

% Get sample landmarks
[i_dLMposTable_TB] = generateLandmarksMap(strShapeModel, Nlandmarks);

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
i_dCAMpos_TB = [1000, 0, 0]';

i32nLandMarks = size(i_dLMposTable_TB, 2);
ui32nTriangles = size(trianglesVertices, 2);

tmpIntersectFlag      = zeros(i32nLandMarks, ui32nTriangles);
intersectDistance     = zeros(i32nLandMarks, ui32nTriangles);
tmpIntersectFlag_MEX  = zeros(i32nLandMarks, ui32nTriangles);
intersectDistance_MEX = zeros(i32nLandMarks, ui32nTriangles);


for idL = 1:i32nLandMarks
    % Compute landmark position unit vector
    dTmpLMpos_TB = i_dLMposTable_TB(2:4, idL); % NOTE: Temporary variable to avoid accessing large array

    dTmpLosCam2LM_TB = dTmpLMpos_TB - i_dCAMpos_TB;
    tmpLosCam2LMnorm = norm(dTmpLosCam2LM_TB);
    dTmpLosCam2LMdir_TB = dTmpLosCam2LM_TB./tmpLosCam2LMnorm;


    ui32nTrianglesInSubset = ui32nTriangles; % TEMPORARY

    ui32TrianglesIDsubset = 1:ui32nTriangles;

    for id = 1:ui32nTrianglesInSubset

        idT = ui32TrianglesIDsubset(id);

        % Get triangle vertices positions
        tmpTriangleVertices = coder.nullcopy(zeros(3, 3));

        i32triangVertPtr = strShapeModel.i32triangVertexPtr(1:3, idT); % Improved mem access speed

        tmpTriangleVertices(1:3, 1) = strShapeModel.dVerticesPos(:, i32triangVertPtr(1) );
        tmpTriangleVertices(1:3, 2) = strShapeModel.dVerticesPos(:, i32triangVertPtr(2) );
        tmpTriangleVertices(1:3, 3) = strShapeModel.dVerticesPos(:, i32triangVertPtr(3) );

        % MATLAB VERSION
%         tic
        [tmpIntersectFlag(idL, idT), ~, ~, intersectDistance(idL, idT)] = fastRayTriangleIntersection(i_dCAMpos_TB, dTmpLosCam2LMdir_TB, ...
            tmpTriangleVertices(:, 1), tmpTriangleVertices(:, 2), tmpTriangleVertices(:, 3));
%         toc


        %% EQUIVALENCE TEST
        % MEX VERSION
%         tic
        [tmpIntersectFlag_MEX(idL, idT), ~, ~, intersectDistance_MEX(idL, idT)] = fastRayTriangleIntersection_MEX(i_dCAMpos_TB, dTmpLosCam2LMdir_TB, ...
            tmpTriangleVertices(:, 1), tmpTriangleVertices(:, 2), tmpTriangleVertices(:, 3));
%         toc


    end

end

outputDiff1 = tmpIntersectFlag_MEX ~= tmpIntersectFlag;
outputDiff2 = intersectDistance_MEX ~= intersectDistance;

sum(abs(outputDiff1), 'all')
sum(abs(outputDiff2), 'all')

