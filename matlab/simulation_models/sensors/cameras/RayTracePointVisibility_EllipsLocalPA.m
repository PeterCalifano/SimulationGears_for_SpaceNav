function [bAllPointsVisibilityMask, dProjectedPoints_UV] = RayTracePointVisibility_EllipsLocalPA(ui32PointsIdx, ...
                                                                                       dPointsPositions_TB, ...
                                                                                       strTargetBodyData, ...
                                                                                       strCameraData, ...
                                                                                       dSunPosition_TB, ...
                                                                                       strFcnOptions, ...
                                                                                       bDEBUG_MODE) %#codegen
arguments
    ui32PointsIdx           (1,:) uint32
    dPointsPositions_TB     (3,:) double
    strTargetBodyData       {isstruct}
    strCameraData           {isstruct} 
    dSunPosition_TB         (3,1) double
    strFcnOptions           {isstruct}
    bDEBUG_MODE             (1,1) logical {islogical} = false
end
%% PROTOTYPE
% [bAllPointsVisibilityMask, dProjectedPoints_UV] = RayTracePointVisibility_EllipsLocalPA(ui32PointsIdx, ...
%                                                                                        dPointsPositions_TB, ...
%                                                                                        strTargetBodyData, ...
%                                                                                        strCameraData, ...
%                                                                                        dSunPosition_TB, ...
%                                                                                        strFcnOptions, ...
%                                                                                        bDEBUG_MODE) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% -Preliminary checks performed:
%   1) Sun illumination of each point considering illumination threshold on dot product. This is equivalent 
%   to considering the local Phase angle using an ellipsoidal shape.
%   2) Heuristic geometrical feasibility check considering dot product of camera position and points
%   positions. This check uses an ellipsoidal shape to prune back-facing points. Do not consider thresholds
%   that are too strict when using irregularly shaped bodies.
%   3) Points projections within field of view using pinhole projection model
% - Intersection check performed using ray tracing to points against each triangle of the mesh.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32PointsIdx       (1,:) uint32
% dPointsPositions_TB (3,:) double
% strTargetBodyData   {isstruct}
% strCameraData       {isstruct}
% dSunPosition_TB      (3,1) double
% strFcnOptions       {isstruct}
% bDEBUG_MODE         (1,1) logical {islogical} = false
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bPointsVisibilityMask
% dProjectedPoints_UV
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-11-2024    Pietro Califano               New optimized version implemented from CheckLMvisibility_rayTrace
% 02-12-2024    Pietro Califano               Unit testing completed, verification and autocoding OK
% 05-07-2025    P. Califano, L. Cesarini.     Fix incorrect visibility conditions for edge cases (points not discarded
%                                             by ray tracing and visible). From Shadow Ray version.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RayTriangleIntersection_MollerTrumbore()
% -------------------------------------------------------------------------------------------------------------
% Defaults
if not(isfield(strFcnOptions, 'bENABLE_HEUR_GEOM_FEAS_CHECK'))
    strFcnOptions.bENABLE_HEUR_GEOM_FEAS_CHECK = false;
end

if not(isfield(strFcnOptions, 'dIllumAngleThr'))
    strFcnOptions.dIllumAngleThr = deg2rad(40); % Valid phase angles: 180-illumAngleThr
end

if not(isfield(strFcnOptions, 'dLosAngleThr')) % Heuristic removal of back-facing points assuming ellipsoid
    strFcnOptions.dLosAngleThr = deg2rad(40); % Valid angles: 180-dLosAngleThr
end

if not(isfield(strFcnOptions, 'bTwoSidedTest')) % Flag to determine MT test type
    strFcnOptions.bTwoSidedTest = true;
end

if not(isfield(strFcnOptions, 'bPointsAreMeshVertices')) % Flag to determine whether points belongs to mesh
    strFcnOptions.bPointsAreMeshVertices = false;
end

bTwoSidedTest           = strFcnOptions.bTwoSidedTest;
bPointsAreMeshVertices  = strFcnOptions.bPointsAreMeshVertices;

% TARGET BODY
assert(size(strTargetBodyData.strShapeModel.ui32triangVertexPtr, 1) == 3, "ERROR: strShapeModel.ui32triangVertexPtr must have [3xN] shape.")
assert(size(strTargetBodyData.strShapeModel.dVerticesPos, 1) == 3, "ERROR: strShapeModel.dVerticesPos must have [3xM] shape.")

ui32triangVertexPtr = strTargetBodyData.strShapeModel.ui32triangVertexPtr;
dVerticesPos        = strTargetBodyData.strShapeModel.dVerticesPos;

dDCM_INfromTB  = strTargetBodyData.dDCM_INfromTB;
dDCM_INfromCAM = strCameraData.dDCM_INfromCAM;

% CAMERA
dCamPosition_IN = strCameraData.dPosition_IN; % TO MODIFY
dResX        = strCameraData.dResX;
dResY        = strCameraData.dResY;

% INPUT ASSERTS (retrocompatibility with MATLAB < 2022b)
assert(size(dPointsPositions_TB, 2) == length(ui32PointsIdx), "ERROR: number of indices must match nunber of points.");

% Compute attitude matrices
% dDCM_TBfromCAM = transpose(dDCM_INfromTB) * dDCM_INfromCAM;
% dDCM_fromTBtoCAM = dDCM_TBfromCAM';

% Compute Sun direction
dSunDir_TB = dSunPosition_TB./norm(dSunPosition_TB);

% Compute camera position in target body fixed
dCamPosition_TB = transpose(dDCM_INfromTB) * dCamPosition_IN;

% Compute thresholds values
dCosIllumAngleThr = cos(strFcnOptions.dIllumAngleThr);
dCosLosAngleThr   = cos(strFcnOptions.dLosAngleThr);

% Get number of triangles and landmarks
ui32NumOfTriangles = uint32(size( ui32triangVertexPtr, 2));
i32NumOfPoints     = int32( size(dPointsPositions_TB, 2) );

% Split array to enable parallelization
% Position
dPointPosX_TB = dPointsPositions_TB(1, :);
dPointPosY_TB = dPointsPositions_TB(2, :);
dPointPosZ_TB = dPointsPositions_TB(3, :);

% Compute all norms using vectorized operation
dPointPosNorms = vecnorm(dPointsPositions_TB(1:3, :), 2, 1);

% Compute direction components
dPointDirX_TB = dPointPosX_TB./dPointPosNorms; 
dPointDirY_TB = dPointPosY_TB./dPointPosNorms; 
dPointDirZ_TB = dPointPosZ_TB./dPointPosNorms; 

% Camera position unit vector in TB
dCameraDir_TB = dCamPosition_TB/norm(dCamPosition_TB);

% Compute dot product with Sun direction (vectorized)
% dPointDirDotSunDir_test = dot([dPointDirX_TB; dPointDirY_TB; dPointDirZ_TB], repmat(-dSunDir_TB, 1,length(ui32PointsIdx)), 1);
dPointDirDotSunDir = sum([dPointDirX_TB; dPointDirY_TB; dPointDirZ_TB]' * (-dSunDir_TB), 2);

% Perform preliminary vectorized checks (get all points that passed checks)
% NOTE: illumi
bIlluminationFeasibilityMask = dPointDirDotSunDir <= 0 | dPointDirDotSunDir <= dCosIllumAngleThr;

% TODO (PC): this heuristic check does not grant the points are actually illuminated, because self-shadowing is not checked using RT (shadow rays). Need to do it as optionally enabled in this function. Essentially, for each visible point from the camera, cast a shadow ray to light source and check for intersections. No intersection --> point is visible.

% Heuristic geometrical check
if strFcnOptions.bENABLE_HEUR_GEOM_FEAS_CHECK == true
    % dPointDirDotCameraPosDir_test = dot(repmat(-dCameraDir_TB, 1,length(ui32PointsIdx)), [dPointDirX_TB; dPointDirY_TB; dPointDirZ_TB], 1);
    dPointDirDotCameraPosDir = sum([dPointDirX_TB; dPointDirY_TB; dPointDirZ_TB]' * (-dCameraDir_TB), 2);
    bGeometricalFeasibilityMask = dPointDirDotCameraPosDir < 0 |  abs(dPointDirDotCameraPosDir) <= dCosLosAngleThr;
else
    bGeometricalFeasibilityMask = true(i32NumOfPoints, 1);
end

% Field of view check
[dProjectedPoints_UV]   = pinholeProjectArrayHP_DCM(strCameraData.dKcam, dDCM_INfromCAM' * dDCM_INfromTB, dCamPosition_TB, dPointsPositions_TB);
bPointWithinFoV = ((dProjectedPoints_UV(1, :) > 0 & dProjectedPoints_UV(1, :) < dResX) & (dProjectedPoints_UV(2, :) > 0 & dProjectedPoints_UV(2, :) < dResY))';

% Get mask to extract points requiring RT to check visibility
bPointsToRayTrace = bPointWithinFoV & bGeometricalFeasibilityMask & bIlluminationFeasibilityMask; 

if bDEBUG_MODE == true
    fprintf("\t Debug print - points visibility check function\n")
    fprintf("\t Not illuminated: %d\n",                      int32(sum(not(bIlluminationFeasibilityMask))) );
    fprintf("\t Geometrically unfeasible (heuristic): %d\n", int32(sum(not(bGeometricalFeasibilityMask))) );
    fprintf("\t Outside field of view: %d\n",      int32(sum(not(bPointWithinFoV))) );
    fprintf("\t To ray trace: %d\n",               int32(sum(bPointsToRayTrace)) );
end

% Extract points to check using ray tracing
dPointPosX_TB = dPointPosX_TB(bPointsToRayTrace);
dPointPosY_TB = dPointPosY_TB(bPointsToRayTrace);
dPointPosZ_TB = dPointPosZ_TB(bPointsToRayTrace);

i32NumOfPointsToTrace = int32( length(dPointPosX_TB) );

% Compute norms of relative positions of points wrt camera
dRayToPointsFromCam_TB  = [dPointPosX_TB; dPointPosY_TB; dPointPosZ_TB] - dCamPosition_TB;
dRayToPointsFromCamNorm = vecnorm(dRayToPointsFromCam_TB, 2, 1);
dRayToPointsFromCam_TB  = dRayToPointsFromCam_TB./dRayToPointsFromCamNorm;

% Compute ray directions (to points)
dPointDirFromCamX_TB = dRayToPointsFromCam_TB(1, :);
dPointDirFromCamY_TB = dRayToPointsFromCam_TB(2, :);
dPointDirFromCamZ_TB = dRayToPointsFromCam_TB(3, :);

% Split arrays for triangular mesh vertices pointers
i32triangVertPtr1 = ui32triangVertexPtr(1, :); % Improved mem access speed
i32triangVertPtr2 = ui32triangVertexPtr(2, :); 
i32triangVertPtr3 = ui32triangVertexPtr(3, :); 

% Field of view check for triangles: create mask to reduce number of intersections
% TODO: review
% bTriangVerticesVisibilityMask
% [dProjectedTriangVert_UV] = pinholeProjectArrayHP_DCM(strCameraData.dKcam, 
%                                                         dDCM_INfromCAM' * dDCM_INfromTB, 
%                                                         dCamPosition_TB, 
%                                                         dVerticesPos);

% bTriangVerticesVisibilityMask = ((dProjectedTriangVert_UV(1, :) > 0 & ...
%                     dProjectedTriangVert_UV(1, :) < dResX) & ...
%                     (dProjectedTriangVert_UV(2, :) > 0 & ...
%                     dProjectedTriangVert_UV(2, :) < dResY))';

%% MAIN computation loop
bPointsVisibilityMask = false(i32NumOfPointsToTrace, 1); % Initialize visibility mask as "no visibility"

% Visibility check for each landmark (position in TB frame)
% TODO add at least one layer of BVH, with a single bounding box for the whole mesh
parfor idL = 1:i32NumOfPointsToTrace
    if bDEBUG_MODE == true 
        if mod(i32NumOfPointsToTrace/idL, 5)
            fprintf("Processing point entry %05d\n", idL);
        end
    end

    %% Intersection computation checks
    % DEVNOTE: may be modified to select only a subset of the triangles for further optimization
    ui32NumTrianglesInSubset = ui32NumOfTriangles;
    ui32TrianglesIDsubset = 1:ui32NumOfTriangles;

    bDoRayTrace = true;
    bNoIntersect = false; % Becomes true only if any triangle is intersected
    dTmpTriangleVertices = coder.nullcopy(zeros(3, 3));

    for id = 1:ui32NumTrianglesInSubset

        idT = ui32TrianglesIDsubset(id);
        % DEVNOTE the only way to avoid broadcast arrays is to compose first vectors cointaining the indices of the vertices for each triangle.
        % Get triangle vertices ptrs
        i32triangVertPtr1_tmp = i32triangVertPtr1(id); % TODO check how to solve broadcasting problem
        i32triangVertPtr2_tmp = i32triangVertPtr2(id);
        i32triangVertPtr3_tmp = i32triangVertPtr3(id);

        if bPointsAreMeshVertices
            % If vertices are used as points, exclude the triangle containing the point as vertices from the intersection check
            bDoRayTrace = all(i32triangVertPtr1_tmp ~= ui32PointsIdx(idL) & ...
                            i32triangVertPtr2_tmp ~= ui32PointsIdx(idL) & ...
                            i32triangVertPtr3_tmp ~= ui32PointsIdx(idL));
        end

        % Check if all three vertices of the triangle are not visible --> continue without testing the idth triangle
        %if all( bTriangVerticesVisibilityMask([i32triangVertPtr1_tmp, i32triangVertPtr2_tmp, i32triangVertPtr3_tmp]) == false )
        %    continue;
        %end

        if bDoRayTrace == true

            % Get triangle vertices positions
            dTmpTriangleVertices(1:3, 1) = dVerticesPos(:, i32triangVertPtr1_tmp ); %#ok<PFBNS>
            dTmpTriangleVertices(1:3, 2) = dVerticesPos(:, i32triangVertPtr2_tmp );
            dTmpTriangleVertices(1:3, 3) = dVerticesPos(:, i32triangVertPtr3_tmp );

            % Perform intersection test against mesh (1 ray 1 triangle) using one-sided test
            % (cull computations excluding back-facing triangles)

            [bTmpIntersectFlag, ~, ~, dIntersectDistance, ~] = RayTriangleIntersection_MollerTrumbore(dCamPosition_TB, ...
                [dPointDirFromCamX_TB(idL); dPointDirFromCamY_TB(idL); dPointDirFromCamZ_TB(idL);],  ...
                dTmpTriangleVertices(:, 1), ...
                dTmpTriangleVertices(:, 2), ...
                dTmpTriangleVertices(:, 3), ...
                bTwoSidedTest, ...
                false); % Normal ray, one-sided test

            bNoIntersect = bNoIntersect || bTmpIntersectFlag;

            %%%%%%%%%%%%%%%%%%%%%% DEBUG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % [bTmpIntersectFlag_check, ~, ~, dIntersectDistance_check] = RayTwoSidedTriangleIntersection_MollerTrembore(dCamPosition_TB, ...
            %     [dPointDirFromCamX_TB(idL); dPointDirFromCamY_TB(idL); dPointDirFromCamZ_TB(idL);],  ...
            %     dTmpTriangleVertices(:, 1), dTmpTriangleVertices(:, 2), dTmpTriangleVertices(:, 3));


            % assert(bTmpIntersectFlag == bTmpIntersectFlag_check)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if (dRayToPointsFromCamNorm(idL) - dIntersectDistance) > eps('single') && bTmpIntersectFlag == true
                % Intersection closer than point detected --> point occluded by mesh (camera does not see it)
                assert(idT <= ui32NumOfTriangles)
                bPointsVisibilityMask(idL) = false;

                %%%%%%%%%%%%%%%%%%%%%% DEBUG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % PlotRays(dCamPosition_TB, [dPointPosX_TB(idL); dPointPosY_TB(idL); dPointPosZ_TB(idL)], dIntersectPoint_TB);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                break;

            elseif (dRayToPointsFromCamNorm(idL) - dIntersectDistance) <= eps('single') && bTmpIntersectFlag == true
                % Intersection farther than point detected --> point visible by camera

                bPointsVisibilityMask(idL) = true;
                % No shadow ray, illumination assumed OK using local PA assumption.
                break;

            elseif id == ui32NumTrianglesInSubset && bNoIntersect == false
                if bPointsVisibilityMask(idL) == false
                    % No intersection detected after testing all triangles and point not occluded by mesh --> point visible
                    bPointsVisibilityMask(idL) = true;
                end
            end

        elseif id == ui32NumTrianglesInSubset && bNoIntersect == false

            % No intersection detected after testing all triangles and point not occluded by mesh AND it is part of the discarded triangles (bDoRayTrace was false)--> point visible
            if bPointsVisibilityMask(idL) == false
                bPointsVisibilityMask(idL) = true;
            end
        end % Ray trace if conditional branch

    end % Loop over triangles

end % Parallelized Loop over points to trace

if bDEBUG_MODE == true
    ui32HowManyVisLM = sum(bPointsVisibilityMask, "all");
    fprintf("\nNumber of VISIBLE points: %d\n", int32(ui32HowManyVisLM));
end

% Map back to all entries
bAllPointsVisibilityMask = bPointsToRayTrace;
bAllPointsVisibilityMask(bPointsToRayTrace == 1) = bPointsVisibilityMask;
dProjectedPoints_UV = dProjectedPoints_UV(:, bAllPointsVisibilityMask);

end




