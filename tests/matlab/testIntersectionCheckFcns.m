close all
clear
clc

% TEST SETUP
% Created by PeterC 01-05-2024
% Complete reworking by PeterC 03-02-2025

% Call parent script to get data
testSimulationSetup();
% [dPointsPositionsGT_TB_check, ui32pointsIDs_check] = objTargetEmulator.GetPointsInTargetFrame(ui32pointsIDs, false);
[dPosVector_W, dRot3_WfromTB] = objTargetEmulator.GetPose();

% Assign data for test
dCameraPosition_TB = [1000; 0; 0];
dSunPosition_TB   = [980; 600; 0];

% dDCM_INfromCAM (NOTE: IN is used here for legacy reasons)
strCamera.dQuat_INfromCAM = simulateTBpointing_PosOnly(dCameraPosition_TB, dPosVector_W, true, false, false);
dDCM_INfromCAM = Quat2DCM(strCamera.dQuat_INfromCAM, true);

strShapeModel = orderfields(strShapeModel);
strTargetBodyData.strShapeModel = strShapeModel;
strTargetBodyData.dDCM_INfromTB = dRot3_WfromTB;

strTargetBodyData = orderfields(strTargetBodyData);

% Camera (object and struct)
dFocalLength = 1E3;
ui32OpticalCentre_uv = [512, 512];
ui32ImageSize = [1024, 1024];

objCameraIntrisics = CCameraIntrinsics(dFocalLength, ui32OpticalCentre_uv, ui32ImageSize);  
disp(objCameraIntrisics)

% Define camera object
objCamera = CProjectiveCamera(objCameraIntrisics);

strCameraData.dDCM_INfromCAM = dDCM_INfromCAM;
strCameraData.dPosition_IN = dCameraPosition_TB; % TO MODIFY
strCameraData.dResX = ui32ImageSize(1);
strCameraData.dResY = ui32ImageSize(2);
strCameraData.dKcam = objCameraIntrisics.K;
strCameraData       = orderfields(strCameraData);

% Generate ray to point
ui32PointID = 1500;

ui32NumTriangles = size(strShapeModel.ui32triangVertexPtr, 2);
dPointPosition_TB = strShapeModel.dVerticesPos(:, ui32PointID);

dRayDirection_TB = dPointPosition_TB - dCameraPosition_TB;
dRayDirection_TB = dRayDirection_TB./norm(dRayDirection_TB);

ui32NumTrianglesInSubset = uint32(ui32NumTriangles); % TEMPORARY
ui32TrianglesIDsubset = uint32(1:ui32NumTriangles);

% Output variables
bIntersectFlag_twoSided = nan(ui32NumTrianglesInSubset, 1);
bIntersectFlag_new      = nan(ui32NumTrianglesInSubset, 1);

dIntersectDistance_twoSided = nan(ui32NumTrianglesInSubset, 1);
dIntersectDistance_new      = nan(ui32NumTrianglesInSubset, 1);

% dTimeit  = zeros(ui32NumTrianglesInSubset, 1);
% dTimeit_new = zeros(ui32NumTrianglesInSubset, 1);

%% RayTwoSidedTriangleIntersection_MollerTrembore
dVerticesPos        = strShapeModel.dVerticesPos;
ui32triangVertexPtr = strShapeModel.ui32triangVertexPtr;
tic
parfor idTriang = 1:ui32NumTrianglesInSubset

    idT = ui32TrianglesIDsubset(idTriang);

    % Get triangle vertices positions
    dTmpTriangleVertices = coder.nullcopy(zeros(3, 3));

    i32triangVertPtr = int32(strShapeModel.ui32triangVertexPtr(1:3, idT)); % Improved mem access speed

    dTmpTriangleVertices(1:3, 1) = strShapeModel.dVerticesPos(:, i32triangVertPtr(1) );
    dTmpTriangleVertices(1:3, 2) = strShapeModel.dVerticesPos(:, i32triangVertPtr(2) );
    dTmpTriangleVertices(1:3, 3) = strShapeModel.dVerticesPos(:, i32triangVertPtr(3) );

    
    [bIntersectFlag_twoSided(idTriang), ~, ~, dIntersectDistance_twoSided(idTriang)] = RayTwoSidedTriangleIntersection_MollerTrembore(dCameraPosition_TB, dRayDirection_TB, ...
        dTmpTriangleVertices(:, 1), dTmpTriangleVertices(:, 2), dTmpTriangleVertices(:, 3));

end
dTimeit = toc;


%% RayTriangleIntersection_MollerTrumbore

tic
parfor idTriang = 1:ui32NumTrianglesInSubset

    idT = ui32TrianglesIDsubset(idTriang);

    % Get triangle vertices positions
    dTmpTriangleVertices = coder.nullcopy(zeros(3, 3));

    i32triangVertPtr = int32(ui32triangVertexPtr(1:3, idT)); % Improved mem access speed

    dTmpTriangleVertices(1:3, 1) = dVerticesPos(:, i32triangVertPtr(1) );
    dTmpTriangleVertices(1:3, 2) = dVerticesPos(:, i32triangVertPtr(2) );
    dTmpTriangleVertices(1:3, 3) = dVerticesPos(:, i32triangVertPtr(3) );


    % RayTriangleIntersection_MollerTrumbore
    [bIntersectFlag_new(idTriang), ~, ~, dIntersectDistance_new(idTriang)] = RayTriangleIntersection_MollerTrumbore_MEX(dCameraPosition_TB, dRayDirection_TB, ...
        dTmpTriangleVertices(:, 1), dTmpTriangleVertices(:, 2), dTmpTriangleVertices(:, 3), true, false);

end

dTimeit_new = toc;


% RayTriangleVectorizedIntersect_MollerTrumbore
% tic
bTwoSidedTest = true;

% Construct arrays of vertices of triangles
dTriangVerticesArray1 = strShapeModel.dVerticesPos(:, strShapeModel.ui32triangVertexPtr(1, :));
dTriangVerticesArray2 = strShapeModel.dVerticesPos(:, strShapeModel.ui32triangVertexPtr(2, :));
dTriangVerticesArray3 = strShapeModel.dVerticesPos(:, strShapeModel.ui32triangVertexPtr(3, :));

[bIsIntersected_newVectorized, ~, ~, ...
    dRangeToIntersection_newVectorized, dIntersectionPoint_newVectorized] = RayTriangleVectorizedIntersect_MollerTrumbore(dCameraPosition_TB, ...
                                                                                                dRayDirection_TB, ...
                                                                                                dTriangVerticesArray1, ...
                                                                                                dTriangVerticesArray2, ...
                                                                                                dTriangVerticesArray3, ...
                                                                                                ui32NumTrianglesInSubset, ...
                                                                                                bTwoSidedTest); 

RayTriangleVectorizedIntersect_MollerTrumbore_ = @() RayTriangleVectorizedIntersect_MollerTrumbore(dCameraPosition_TB, ...
                                                                                    dRayDirection_TB, ...
                                                                                    dTriangVerticesArray1, ...
                                                                                    dTriangVerticesArray2, ...
                                                                                    dTriangVerticesArray3, ...
                                                                                    ui32NumTrianglesInSubset, ...
                                                                                    bTwoSidedTest); 

% dTimeit_newVectorized = toc;

% RayTriangleVectorizedIntersect_MollerTrumbore with rejection masks
% tic
% [bIsIntersected_newVectorizedRej, ~, ~, ...
    % dRangeToIntersection_newVectorizedRej, dIntersectionPoint_newVectorizedRej] = RayTriangleVectorizedIntersectWithReject_MollerTrumbore(dCameraPosition_TB, ...
    %                                                                                             dRayDirection_TB, ...
    %                                                                                             dTriangVerticesArray1, ...
    %                                                                                             dTriangVerticesArray2, ...
    %                                                                                             dTriangVerticesArray3, ...
    %                                                                                             ui32NumTrianglesInSubset, ...
    %                                                                                             bTwoSidedTest); 
% dTimeit_newVectorizedRej = toc;

RayTriangleVectorizedIntersectWithReject_MollerTrumbore_ = @() RayTriangleVectorizedIntersectWithReject_MollerTrumbore(dCameraPosition_TB, ...
                                                                                    dRayDirection_TB, ...
                                                                                    dTriangVerticesArray1, ...
                                                                                    dTriangVerticesArray2, ...
                                                                                    dTriangVerticesArray3, ...
                                                                                    ui32NumTrianglesInSubset, ...
                                                                                    bTwoSidedTest); 

% [bIsIntersected, dUbarycenCoord, dVbarycenCoord, ...
%           dRangeToIntersection, dIntersectionPoint] = RayTriangleVectorized_byGPT(dCameraPosition_TB, ...
%                                                                                                 dRayDirection_TB, ...
%                                                                                                 dTriangVerticesArray1, ...
%                                                                                                 dTriangVerticesArray2, ...
%                                                                                                 dTriangVerticesArray3, ...
%                                                                                                 ui32NumTrianglesInSubset, ...
%                                                                                                 bTwoSidedTest); 


% Display info
dAvgTime_vect = AverageFunctionTiming(RayTriangleVectorizedIntersect_MollerTrumbore_, 10000);
dAvgTime_vectMasked = AverageFunctionTiming(RayTriangleVectorizedIntersect_MollerTrumbore_, 10000);

disp( dTimeit );
disp( dTimeit_new );
disp(dAvgTime_vect);
disp(dAvgTime_vectMasked);

bDifferenceMask = bIntersectFlag_new ~= bIntersectFlag_twoSided;
sum(bDifferenceMask)

% Check results of vectorized against for loop version
dIntersectMinDist = min(dIntersectDistance_twoSided(dIntersectDistance_twoSided > 0));
dIntersectMinDist_new = min(dIntersectDistance_new(dIntersectDistance_new > 0));



