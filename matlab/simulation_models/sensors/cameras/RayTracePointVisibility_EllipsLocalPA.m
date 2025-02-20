function [bAllPointsVisibilityMask, dProjectedPoints_UV] = RayTracePointVisibility_EllipsLocalPA(ui32PointsIdx, ...
                                                                                       dPointsPositions_TB, ...
                                                                                       strTargetBodyData, ...
                                                                                       strCameraData, ...
                                                                                       dSunDir_TB, ...
                                                                                       strFcnOptions, ...
                                                                                       bDEBUG_MODE) %#codegen
arguments
    ui32PointsIdx       (1,:) uint32
    dPointsPositions_TB (3,:) double
    strTargetBodyData   {isstruct}
    strCameraData       {isstruct} 
    dSunDir_TB          (3,1) double
    strFcnOptions       {isstruct}
    bDEBUG_MODE         (1,1) logical {islogical} = false
end
%% PROTOTYPE
% 
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
% dSunDir_TB          (3,1) double
% strFcnOptions       {isstruct}
% bDEBUG_MODE         (1,1) logical {islogical} = false
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bPointsVisibilityMask
% dProjectedPoints_UV
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 30-11-2024    Pietro Califano    New optimized version implemented from CheckLMvisibility_rayTrace
% 02-12-2024    Pietro Califano    Unit testing completed, verification and autocoding OK
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% 1) fastRayTriangleIntersection()
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 
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


% TARGET BODY
assert(size(strTargetBodyData.strShapeModel.ui32triangVertexPtr, 1) == 3, "ERROR: strShapeModel.ui32triangVertexPtr must have [3xN] shape.")
assert(size(strTargetBodyData.strShapeModel.dVerticesPos, 1) == 3, "ERROR: strShapeModel.dVerticesPos must have [3xM] shape.")

ui32triangVertexPtr = strTargetBodyData.strShapeModel.ui32triangVertexPtr;
dVerticesPos        = strTargetBodyData.strShapeModel.dVerticesPos;

dDCM_INfromTB  = strTargetBodyData.dDCM_INfromTB;
dDCM_INfromCAM = strCameraData.dDCM_INfromCAM;

% CAMERA
dPosition_IN = strCameraData.dPosition_IN; % TO MODIFY
dResX        = strCameraData.dResX;
dResY        = strCameraData.dResY;

% INPUT ASSERTS (retrocompatibility with MATLAB < 2022b)
assert(size(dPointsPositions_TB, 2) == length(ui32PointsIdx), "ERROR: number of indices must match nunber of points.");

% Compute attitude matrices
% dDCM_TBfromCAM = transpose(dDCM_INfromTB) * dDCM_INfromCAM;
% dDCM_fromTBtoCAM = dDCM_TBfromCAM';

% Ensure Sun direction is normalized
if any(abs(dSunDir_TB) > 1.0, 'all')
    dSunDir_TB = dSunDir_TB./norm(dSunDir_TB);
end

% Compute camera position in target body fixed
dPosition_TB = transpose(dDCM_INfromTB) * dPosition_IN;

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
dCameraDir_TB = dPosition_TB/norm(dPosition_TB);

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
[dProjectedPoints_UV]   = pinholeProjectArrayHP_DCM(strCameraData.dKcam, dDCM_INfromCAM' * dDCM_INfromTB, dPosition_TB, dPointsPositions_TB);
bPointWithinFoV = ((dProjectedPoints_UV(1, :) > 0 & dProjectedPoints_UV(1, :) < dResX) & (dProjectedPoints_UV(2, :) > 0 & dProjectedPoints_UV(2, :) < dResY))';

% Get mask to extract points requiring RT to check visibility
bPointsToRayTrace = bPointWithinFoV & bGeometricalFeasibilityMask & bIlluminationFeasibilityMask; 

if bDEBUG_MODE == true
    fprintf("\t Debug print - points visibility check function\n")
    fprintf("\t Not illuminated: %d\n",                      int32(sum(not(bIlluminationFeasibilityMask))) );
    fprintf("\t Geometrically unfeasible (heuristic): %d\n", int32(sum(not(bGeometricalFeasibilityMask))) );
    fprintf("\t Outside field of view: %d\n",                int32(sum(not(bPointWithinFoV))) );
    fprintf("\t Remaining to ray trace: %d\n",               int32(sum(bPointsToRayTrace)) );
end

% Extract points to check using ray tracing
dPointPosX_TB = dPointPosX_TB(bPointsToRayTrace);
dPointPosY_TB = dPointPosY_TB(bPointsToRayTrace);
dPointPosZ_TB = dPointPosZ_TB(bPointsToRayTrace);

i32NumOfPointsToTrace = int32( length(dPointPosX_TB) );

% Compute norms of relative positions of points wrt camera
dRayToPointsFromCam_TB  = [dPointPosX_TB; dPointPosY_TB; dPointPosZ_TB] - dPosition_TB;
dRayToPointsFromCamNorm = vecnorm(dRayToPointsFromCam_TB, 2, 1);
dRayToPointsFromCam_TB  = dRayToPointsFromCam_TB./dRayToPointsFromCamNorm;

dPointDirFromCamX_TB = dRayToPointsFromCam_TB(1, :);
dPointDirFromCamY_TB = dRayToPointsFromCam_TB(2, :);
dPointDirFromCamZ_TB = dRayToPointsFromCam_TB(3, :);

% Split arrays for triangular mesh vertices pointers
i32triangVertPtr1 = ui32triangVertexPtr(1, :); % Improved mem access speed
i32triangVertPtr2 = ui32triangVertexPtr(2, :); 
i32triangVertPtr3 = ui32triangVertexPtr(3, :); 

%% MAIN computation loop
bPointsVisibilityMask = false(i32NumOfPointsToTrace, 1); % Initialize visibility mask as "no visibility"

% Visibility check for each landmark (position in TB frame)
parfor idL = 1:i32NumOfPointsToTrace
    if bDEBUG_MODE == true 
        if mod(i32NumOfPointsToTrace/idL, 5)
            fprintf("Processing point entry %05d\n", idL)
        end
    end


    %% Intersection computation checks
    % DEVNOTE: may be modified to select only a subset of the triangles for further optimization
    ui32NumTrianglesInSubset = ui32NumOfTriangles; 
    ui32TrianglesIDsubset = 1:ui32NumOfTriangles;
    bIsIntersected = false(ui32NumTrianglesInSubset, 1);

    for id = 1:ui32NumTrianglesInSubset

        idT = ui32TrianglesIDsubset(id);
        % DEVNOTE the only way to avoid broadcast arrays is to compose first vectors cointaining the indices of the vertices for each triangle.
        % Get triangle vertices ptrs
        i32triangVertPtr1_tmp = i32triangVertPtr1(id);
        i32triangVertPtr2_tmp = i32triangVertPtr2(id);
        i32triangVertPtr3_tmp = i32triangVertPtr3(id);

        % Get triangle vertices positions
        tmpTriangleVertices = coder.nullcopy(zeros(3, 3));

        if all(i32triangVertPtr1_tmp ~= ui32PointsIdx(idL) & i32triangVertPtr2_tmp ~= ui32PointsIdx(idL) & ...
                i32triangVertPtr3_tmp ~= ui32PointsIdx(idL))

            tmpTriangleVertices(1:3, 1) = dVerticesPos(:, i32triangVertPtr1_tmp ); %#ok<PFBNS>
            tmpTriangleVertices(1:3, 2) = dVerticesPos(:, i32triangVertPtr2_tmp );
            tmpTriangleVertices(1:3, 3) = dVerticesPos(:, i32triangVertPtr3_tmp );

            % Evaluate intersection through ray tracing

            [bTmpIntersectFlag, ~, ~, dIntersectDistance] = RayTwoSidedTriangleIntersection_MollerTrembore(dPosition_TB, ...
                [dPointDirFromCamX_TB(idL); dPointDirFromCamY_TB(idL); dPointDirFromCamZ_TB(idL);],  ... 
                tmpTriangleVertices(:, 1), tmpTriangleVertices(:, 2), tmpTriangleVertices(:, 3));

            if (dRayToPointsFromCamNorm(idL) - dIntersectDistance) > eps('single') && bTmpIntersectFlag == true
                assert(idT <= ui32NumOfTriangles)
                bIsIntersected(idT) = true;

                % TODO (PC) shadow ray casting goes here, to check for intersection with mesh (except for the triangles of which the vertex is part of). Triangles may be excluded easily based on some conditions here.

                break; % Intersection detected --> no need to check other triangles
            end
        end

    end

    if any(bIsIntersected == true, 1)
        % INTERSECTION --> LM NOT VISIBLE
    else
        % NO INTERSECTION --> LM MARKED AS VISIBLE
        assert(idL <= i32NumOfPoints)
        bPointsVisibilityMask(idL) = true;
        % NOTE: Default is false. All selected triangles must be checked to declare visibility
    end

end

if bDEBUG_MODE == true
    ui32HowManyVisLM = sum(bPointsVisibilityMask, "all");
    fprintf("\nNumber of VISIBLE points: %d\n", int32(ui32HowManyVisLM));
end

% Map back to all entries
bAllPointsVisibilityMask = bPointsToRayTrace;
bAllPointsVisibilityMask(bPointsToRayTrace == 1) = bPointsVisibilityMask;
dProjectedPoints_UV = dProjectedPoints_UV(:, bAllPointsVisibilityMask);

end



