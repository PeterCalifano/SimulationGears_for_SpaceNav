function [bAllPointsVisibilityMask, dProjectedPoints_UV] = ParallelRayTracePointVisibility_ShadowRays(ui32PointsIdx, ...
                                                                                               dPointsPositions_TB, ...
                                                                                               strTargetBodyData, ...
                                                                                               strCameraData, ...
                                                                                               dSunPosition_TB, ...
                                                                                               bDEBUG_MODE, ...
                                                                                               bTwoSidedTest) %#codegen
arguments
    ui32PointsIdx       (1,:) uint32
    dPointsPositions_TB (3,:) double
    strTargetBodyData   {isstruct}
    strCameraData       {isstruct} 
    dSunPosition_TB     (3,1) double
    bDEBUG_MODE         (1,1) logical {islogical} = false
    bTwoSidedTest       (1,1) logical {islogical} = false;
end
%% PROTOTYPE
% 
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% DEVNOTE: this version uses parallel.pool.Constant to try to decrease compute time. However, preliminary
% tests seem to indicate that it has the opposite effect in this design.
% -Preliminary checks performed:
%   1) Points projections within field of view using pinhole projection model
% - Intersection check performed using ray tracing to points against each triangle of the mesh, one-sided to
%   automatically exclude back-facing triangles (normals in the same direction as the ray, withopaque body)
% - Illumination check performed using a shadow ray from point to light. Point is illuminated if shadow ray 
%   to light does not intersect anything. Illumination check may be pre-computed if light is fixed (not done
%   here). All points that pass both intersection checks are marked as visible.
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
% 30-11-2024    Pietro Califano     New optimized version implemented from CheckLMvisibility_rayTrace
% 02-12-2024    Pietro Califano     Unit testing completed, verification and autocoding OK
% 03-02-2025    Pietro Califano     Implement new version with shadow rays tailored for parallel toolbox     
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% 1) fastRayTriangleIntersection()
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 
% -------------------------------------------------------------------------------------------------------------

% Defaults

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
% assert(size(dPointsPositions_TB, 2) == length(ui32PointsIdx), "ERROR: number of indices must match nunber of points.");

% Compute attitude matrices
% dDCM_TBfromCAM = transpose(dDCM_INfromTB) * dDCM_INfromCAM;
% dDCM_fromTBtoCAM = dDCM_TBfromCAM';

% dSunDir_TB = dSunPosition_TB./norm(dSunPosition_TB);

% Compute camera position in target body fixed
dCamPosition_TB = transpose(dDCM_INfromTB) * dCamPosition_IN;

% Get number of triangles and landmarks
ui32NumOfTriangles = uint32(size( ui32triangVertexPtr, 2));
% i32NumOfPoints     = int32( size(dPointsPositions_TB, 2) );

% Split array to enable parallelization
% Position
dPointPosX_TB = dPointsPositions_TB(1, :);
dPointPosY_TB = dPointsPositions_TB(2, :);
dPointPosZ_TB = dPointsPositions_TB(3, :);

% Camera position unit vector in TB
% dCameraDir_TB = dPosition_TB/norm(dPosition_TB);

% Compute dot product with Sun direction (vectorized)
% dPointDirDotSunDir_test = dot([dPointDirX_TB; dPointDirY_TB; dPointDirZ_TB], repmat(-dSunPosition_TB, 1,length(ui32PointsIdx)), 1);
% dPointDirDotSunDir = sum([dPointDirX_TB; dPointDirY_TB; dPointDirZ_TB]' * (-dSunPosition_TB), 2);

% Field of view check
[dProjectedPoints_UV] = pinholeProjectArrayHP_DCM(strCameraData.dKcam, dDCM_INfromCAM' * dDCM_INfromTB, dCamPosition_TB, dPointsPositions_TB);
bPointWithinFoV = ((dProjectedPoints_UV(1, :) > 0 & dProjectedPoints_UV(1, :) < dResX) & (dProjectedPoints_UV(2, :) > 0 & dProjectedPoints_UV(2, :) < dResY))';

% Get mask to extract points requiring RT to check visibility
bPointsToRayTrace = bPointWithinFoV; 

if bDEBUG_MODE == true
    fprintf("\t Debug print - points visibility check function\n")
    %fprintf("\t Not illuminated: %d\n",                      int32(sum(not(bIlluminationFeasibilityMask))) );
    %fprintf("\t Geometrically unfeasible (heuristic): %d\n", int32(sum(not(bGeometricalFeasibilityMask))) );
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

% Redefine arrays as constant parpool objects
dPointPosX_TB = parallel.pool.Constant(dPointPosX_TB);
dPointPosY_TB = parallel.pool.Constant(dPointPosY_TB);
dPointPosZ_TB = parallel.pool.Constant(dPointPosZ_TB);

i32triangVertPtr1 = parallel.pool.Constant(i32triangVertPtr1);
i32triangVertPtr2 = parallel.pool.Constant(i32triangVertPtr2);
i32triangVertPtr3 = parallel.pool.Constant(i32triangVertPtr3);

dPointDirFromCamX_TB = parallel.pool.Constant( dPointDirFromCamX_TB );
dPointDirFromCamY_TB = parallel.pool.Constant( dPointDirFromCamY_TB );
dPointDirFromCamZ_TB = parallel.pool.Constant( dPointDirFromCamZ_TB );

dVerticesPos        = parallel.pool.Constant(dVerticesPos);

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
    
    % Loop over triangles (TODO attempt to vectorize code)
    for id = 1:ui32NumTrianglesInSubset

        idT = ui32TrianglesIDsubset(id);
        % DEVNOTE the only way to avoid broadcast arrays is to compose first vectors cointaining the indices of the vertices for each triangle.
        % Get triangle vertices ptrs
        i32triangVertPtr1_tmp = i32triangVertPtr1.Value(id); % TODO check how to solve broadcasting problem
        i32triangVertPtr2_tmp = i32triangVertPtr2.Value(id);
        i32triangVertPtr3_tmp = i32triangVertPtr3.Value(id);

        % Get triangle vertices positions
        dTmpTriangleVertices = coder.nullcopy(zeros(3, 3));

        if all(i32triangVertPtr1_tmp ~= ui32PointsIdx(idL) & i32triangVertPtr2_tmp ~= ui32PointsIdx(idL) & ...
                i32triangVertPtr3_tmp ~= ui32PointsIdx(idL))

            dTmpTriangleVertices(1:3, 1) = dVerticesPos.Value(:, i32triangVertPtr1_tmp ); 
            dTmpTriangleVertices(1:3, 2) = dVerticesPos.Value(:, i32triangVertPtr2_tmp );
            dTmpTriangleVertices(1:3, 3) = dVerticesPos.Value(:, i32triangVertPtr3_tmp );

            % Perform intersection test against mesh (1 ray 1 triangle) using one-sided test 
            % (cull computations excluding back-facing triangles)
    
            [bTmpIntersectFlag, ~, ~, dIntersectDistance] = RayTriangleIntersection_MollerTrumbore(dCamPosition_TB, ...
                [dPointDirFromCamX_TB.Value(idL); dPointDirFromCamY_TB.Value(idL); dPointDirFromCamZ_TB.Value(idL);],  ... 
                dTmpTriangleVertices(:, 1), ...
                dTmpTriangleVertices(:, 2), ...
                dTmpTriangleVertices(:, 3), ...
                bTwoSidedTest, ...
                false); % Normal ray, one-sided test


            if (dRayToPointsFromCamNorm(idL) - dIntersectDistance) > eps('single') && bTmpIntersectFlag == true
                % Intersection closer than point detected --> point occluded by mesh (camera does not see it)
                assert(idT <= ui32NumOfTriangles)
                bPointsVisibilityMask(idL) = false;
                break; 

            elseif (dRayToPointsFromCamNorm(idL) - dIntersectDistance) <= eps('single') && bTmpIntersectFlag == true
                % Intersection farther than point detected --> point visible by camera
                % Cast shadow ray to light and check for illumination occlusion
                
                % Compute ray direction (to Sun)
                dSunDirFromPoint_TB = dSunPosition_TB - [dPointPosX_TB.Value(idL); dPointPosY_TB.Value(idL); dPointPosZ_TB.Value(idL)];
                dSunDirFromPoint_TB = dSunDirFromPoint_TB./norm(dSunDirFromPoint_TB);

                % NOTE: for loop over all triangles of mesh inside (one-sided test)
                [bLightOcclusion] = TraceShadowRay([dPointPosX_TB.Value(idL); dPointPosY_TB.Value(idL); dPointPosZ_TB.Value(idL)], ...
                                            dSunDirFromPoint_TB, ...
                                            dVerticesPos, ...
                                            ui32NumTrianglesInSubset, ...
                                            i32triangVertPtr1, ...
                                            i32triangVertPtr2, ...
                                            i32triangVertPtr3, ...
                                            ui32PointsIdx(idL));

                bPointsVisibilityMask(idL) = not(bLightOcclusion); % Point is visible if light not occluded

            elseif bTmpIntersectFlag == false && bTwoSidedTest == false
                % One-sided test may return no intersection because computation has been culled.
                continue;

            elseif bTmpIntersectFlag == false && bTwoSidedTest == true
                warning( bTmpIntersectFlag, sprintf('No intersection with mesh found for point %d. However this should have not occurred because points to check belong to mesh!', idL) ); %#ok<SPWRN>
                bPointsVisibilityMask(idL) = false;

            end
        end

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

% LOCAL FUNCTION for Shadow Ray check
function [bLightOcclusion] = TraceShadowRay(dRayOrigin, ...
                                            dRayDirection, ...
                                            dVerticesPos, ...
                                            ui32NumTrianglesInSubset, ...
                                            i32triangVertPtr1, ...
                                            i32triangVertPtr2, ...
                                            i32triangVertPtr3, ...
                                            ui32TestPointIdx)

bLightOcclusion = false;
% Loop over triangles (TODO attempt to vectorize code)
for id = 1:ui32NumTrianglesInSubset

    % DEVNOTE the only way to avoid broadcast arrays is to compose first vectors cointaining the indices of the vertices for each triangle.
    % Get triangle vertices ptrs
    i32triangVertPtr1_tmp = i32triangVertPtr1.Value(id); % TODO check how to solve broadcasting problem
    i32triangVertPtr2_tmp = i32triangVertPtr2.Value(id);
    i32triangVertPtr3_tmp = i32triangVertPtr3.Value(id);

    % Get triangle vertices positions
    dTmpTriangleVertices = coder.nullcopy(zeros(3, 3));

    if all(i32triangVertPtr1_tmp ~= ui32TestPointIdx & i32triangVertPtr2_tmp ~= ui32TestPointIdx & ...
            i32triangVertPtr3_tmp ~= ui32TestPointIdx)

        dTmpTriangleVertices(1:3, 1) = dVerticesPos.Value(:, i32triangVertPtr1_tmp );
        dTmpTriangleVertices(1:3, 2) = dVerticesPos.Value(:, i32triangVertPtr2_tmp );
        dTmpTriangleVertices(1:3, 3) = dVerticesPos.Value(:, i32triangVertPtr3_tmp );

        % Perform intersection test against mesh (1 ray 1 triangle) using one-sided test
        % (cull computations excluding back-facing triangles)
        [bLightOcclusion] = RayTriangleIntersection_MollerTrumbore(dRayOrigin, ...
                                                                   dRayDirection,  ...
                                                                   dTmpTriangleVertices(:, 1), ...
                                                                   dTmpTriangleVertices(:, 2), ...
                                                                   dTmpTriangleVertices(:, 3), ...
                                                                   false, ...
                                                                   true); % Shadow ray, one-sided test

        if bLightOcclusion == true
            break; % Intersection detected by Shadow Ray --> light occluded by mesh, point not illuminated
        end

    end

end

end


