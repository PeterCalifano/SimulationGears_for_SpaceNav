function [bIsLMvisibleMask] = CheckLMvisibility_rayTrace(dPointsPositions_TB, ...
    strTargetBody, strCamera, dSunDir_TB, strFcnOptions) %#codegen
arguments
    dPointsPositions_TB   
    strTargetBody
    strCamera
    dSunDir_TB
    strFcnOptions
end
%% PROTOTYPE
% [bIsLMvisibleMask] = CheckLMvisibility_rayTrace(dLMposTable_TB, ...
%     strTargetBody, strCamera, dSunDir_TB, strFcnOptions) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% Checks performed by CheckLMvisibility_rayTrace
% 1) Sun illumination
% 2) Line of sight to landmark insersection with shape model
% 3) Line of sight within camera field of view
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dLMposTable_TB
% strTargetBody
% strCamera
% dSunDir_TB
% strFcnOptions
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bIsLMvisibleMask
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-04-2024        Pietro Califano         First version coded.
% 16-04-2024        Pietro Califano         Modified to include FoV check and struct() as inputs
% 19-04-2024        Pietro Califano         Bug fixes and added DEBUG_ON flag for printing
% 22-04-2024        Pietro Califano         Function modified for codegen
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% 1) fastRayTriangleIntersection()
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Additional conditions to reduce number of intersections to check
%    (triangles subset extraction conditions)
% -------------------------------------------------------------------------------------------------------------
%% Function code
% Get data from structures
% OPTIONS
i_dIllumAngleThr = strFcnOptions.dIllumAngleThr;
i_dLosAngleThr   = strFcnOptions.dLosAngleThr;

% TARGET BODY
i_strShapeModel  = strTargetBody.strShapeModel;
dQuat_INfromTB   = strTargetBody.dQuat_INfromTB;
bIS_JPL_QUAT_TB  = strTargetBody.bIS_JPL_QUAT;

% CAMERA
dCAMpos_IN = strCamera.dCAMpos_IN; % TO MODIFY
dFovX      = strCamera.dFovX;
dFovY      = strCamera.dFovY;

dQuat_INfromCAM = strCamera.dQuat_INfromCAM;
bIS_JPL_QUAT_CAM = strCamera.bIS_JPL_QUAT;

DEBUG_ON = false;

% INPUT ASSERTS (retrocompatibility with MATLAB < 2022b)
assert(size(dPointsPositions_TB, 1) == 4, "ERROR: i_dLMposTable_TB must have [4xL] shape (row1: LMs ids)");

assert( isfield(i_strShapeModel, "ui32triangVertexPtr") , "ERROR: missing field in input struct");
assert( isfield(i_strShapeModel, "dVerticesPos") , "ERROR: missing field in input struct");

assert(size(i_strShapeModel.ui32triangVertexPtr, 1) == 3, "ERROR: i_strShapeModel.ui32triangVertexPtr must have [3xN] shape.")
assert(size(i_strShapeModel.dVerticesPos, 1) == 3, "ERROR: i_strShapeModel.dVerticesPos must have [3xM] shape.")
% assert( all(size(dTargetPos_IN) == [3,1], "all") )
assert( all(size(dCAMpos_IN) == [3,1], "all") )

% Compute attitude matrices
dDCM_INfromTB  = Quat2DCM(dQuat_INfromTB, bIS_JPL_QUAT_TB);
dDCM_INfromCAM = Quat2DCM(dQuat_INfromCAM, bIS_JPL_QUAT_CAM);
dDCM_TBfromCAM = transpose(dDCM_INfromTB) * dDCM_INfromCAM;

dDCM_fromTBtoCAM = dDCM_TBfromCAM';

% NOTE: Camera boresight is assumed to be +Z axis
% dBoresightCAM_TB = dDCM_fromCAMtoTB * [0;0;1];

% Compute camera position in target body fixed
i_dCAMpos_TB = transpose(dDCM_INfromTB) * dCAMpos_IN;

% Compute thresholds values
dcosIllumAngleThr = cos(i_dIllumAngleThr);
dcosLosAngleThr = cos(i_dLosAngleThr);
% dcosFovX = cos(1.025*dFovX); % Added +2.5% to account for the correct prospective geometry
%                               (distance from projection plane is not unitary expect on the boresight)
% dcosFovY = cos(1.025*dFovY);

% Get number of triangles and landmarks
ui32nTriangles = uint32(size( i_strShapeModel.ui32triangVertexPtr, 2));
i32nLandMarks = int32( size(dPointsPositions_TB, 2) );

% Camera position unit vector in TB
i_dCAMposUnitvec_TB = i_dCAMpos_TB/norm(i_dCAMpos_TB);

%% MAIN computation loop
bIsLMvisibleMask = false(i32nLandMarks, 1);

% Visibility check for each landmark in LMposTable (position in TB frame)
for idL = 1:i32nLandMarks
    if DEBUG_ON == true
        fprintf("LM ID: %d", idL);
    end
    % Compute landmark position unit vector
    dTmpLMpos_TB = dPointsPositions_TB(2:4, idL); % NOTE: Temporary variable to avoid accessing large array
    dTmpLMdir_TB = dTmpLMpos_TB/norm(dTmpLMpos_TB);

    dLMdirDotSundir = dot(dTmpLMdir_TB, -dSunDir_TB);

    % Check if LM is illuminated
    if dLMdirDotSundir <= 0 || dLMdirDotSundir <= dcosIllumAngleThr

        % COmpute projection of landmark position along camera position unit vector
        dLMdirDotPosSCunitVec = dot(-i_dCAMposUnitvec_TB, dTmpLMdir_TB);

        % Geometrically check that point is not behind (not visible)
        if (dLMdirDotPosSCunitVec < 0) || ( abs(dLMdirDotPosSCunitVec) <= dcosLosAngleThr ) 

            dTmpLosCam2LM_TB = dTmpLMpos_TB - i_dCAMpos_TB;
            tmpLosCam2LMnorm = norm(dTmpLosCam2LM_TB);
            dTmpLosCam2LMdir_TB = dTmpLosCam2LM_TB./tmpLosCam2LMnorm;

            dTmpLosCam2LM_CAM = dDCM_fromTBtoCAM * dTmpLosCam2LMdir_TB;
            dTmpLosCam2LM_CAM = dTmpLosCam2LM_CAM./norm(dTmpLosCam2LM_CAM);

            % Los projection onto plane XZ --> check FovY
            % dTmpLosCam2LMxz_CAM = [dTmpLosCam2LM_CAM(1); 0; dTmpLosCam2LM_CAM(3)];
            % dTmpLosCam2LMxz_CAM = dTmpLosCam2LMxz_CAM./norm(dTmpLosCam2LMxz_CAM);

            % Los projection onto plane YZ --> check FovX
            % dTmpLosCam2LMyz_CAM = [0; dTmpLosCam2LM_CAM(2:3)];
            % dTmpLosCam2LMyz_CAM = dTmpLosCam2LMyz_CAM./norm(dTmpLosCam2LMyz_CAM);

            % dLosDotLosSC2LMxz_Fovy = dot(dTmpLosCam2LM_CAM, dTmpLosCam2LMxz_CAM);
            % dLosDotLosSC2LMyz_Fovx = dot(dTmpLosCam2LM_CAM, dTmpLosCam2LMyz_CAM);
            losInZYplane = [0; dTmpLosCam2LM_CAM(2:3)];
            losInZYplane = losInZYplane./norm(losInZYplane);

            thetaEl = abs(asin(dTmpLosCam2LM_CAM(1)));
            thetaAz = abs(asin(losInZYplane(2)));

            % DEVNOTE: 
            % error('Ma come cazzo hai fatto a pensare che questo check sia meglio che proiettare sul piano immagine?')

            % thetaAz = acos(dot(dTmpLosCam2LM_CAM, [0,0,1]));
            % thetaEl = thetaAz;
            % Geometrically check if Landmark is inside the field of view of the camera
            % if  (dLosDotLosSC2LMyz_Fovx > dcosFovX) && ...
            %         (dLosDotLosSC2LMxz_Fovy > dcosFovY)

            if  (thetaEl < dFovY) && (thetaAz < dFovX)

                %% Intersection computation checks
                % ALL VISIBILITY CONDITIONS ARE MET --> CHECK INTERSECTION

                % TODO: modify to select only a subset of the triangles
                ui32nTrianglesInSubset = ui32nTriangles; % TEMPORARY
                
                ui32TrianglesIDsubset = 1:ui32nTriangles;
                bIsIntersected = false(ui32nTriangles, 1);

                for id = 1:ui32nTrianglesInSubset

                    idT = ui32TrianglesIDsubset(id);

                    % Get triangle vertices positions
                    tmpTriangleVertices = coder.nullcopy(zeros(3, 3));

                    i32triangVertPtr = i_strShapeModel.ui32triangVertexPtr(1:3, idT); % Improved mem access speed

                    if all(i32triangVertPtr ~= idL)

                        tmpTriangleVertices(1:3, 1) = i_strShapeModel.dVerticesPos(:, i32triangVertPtr(1) );
                        tmpTriangleVertices(1:3, 2) = i_strShapeModel.dVerticesPos(:, i32triangVertPtr(2) );
                        tmpTriangleVertices(1:3, 3) = i_strShapeModel.dVerticesPos(:, i32triangVertPtr(3) );

                        % Evaluate intersection through ray tracing
                       
                        [tmpIntersectFlag, ~, ~, intersectDistance] = RayTwoSidedTriangleIntersection_MollerTrembore(i_dCAMpos_TB, dTmpLosCam2LMdir_TB, ...
                            tmpTriangleVertices(:, 1), tmpTriangleVertices(:, 2), tmpTriangleVertices(:, 3));

                        if (tmpLosCam2LMnorm - intersectDistance) > eps('single') && tmpIntersectFlag == true
                            assert(idT <= ui32nTriangles)
                            bIsIntersected(idT) = true;
                            break; % Intersection detected --> no need to check other triangles

%                         elseif tmpIntersectFlag == true && intersectDistance <= tmpLosCam2LMnorm
%                             bIsIntersected(idT) = false; % Not really needed if default == false
                        end
                    end

                end

                if any(bIsIntersected == true, 1)
                    % INTERSECTION --> LM NOT VISIBLE
                else
                    % NO INTERSECTION --> LM MARKED AS VISIBLE
                    assert(idL <= i32nLandMarks)
                    bIsLMvisibleMask(idL) = true;
                    % NOTE: Default is false. All selected triangles must be checked to declare visibility
                end

                if DEBUG_ON == true

                    if bIsLMvisibleMask(idL) == false
                        fprintf(" --> NOT VIS: INTERSECTION\n")
                    else
                        fprintf(" --> VISIBLE: NO INTERSECTION\n")
                    end
                end

            else
                % NOT VISIBLE DUE TO LM OUT OF CAMERA FOV
                if DEBUG_ON == true
                    fprintf(" --> NOT VIS: OUT OF FOV\n")
                end
            end
        else
            % NOT VISIBLE DUE TO LM DECLARED "BEHIND" GEOMETRICALLY
            if DEBUG_ON == true
                fprintf(" --> NOT VIS: BEHIND\n")
            end
        end
    else
        % NOT VISIBLE DUE TO SUN ILLUMINATION
        if DEBUG_ON == true
            fprintf(" --> NOT VIS: NO SUN\n")
        end
    end

end

if DEBUG_ON == true
    howManyVisLM = sum(bIsLMvisibleMask, "all");
    fprintf("\nNumber of VISIBLE LM: %d\n", howManyVisLM);
end



end
