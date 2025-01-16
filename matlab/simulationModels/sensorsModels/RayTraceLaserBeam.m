function [bInsersectionFlag, dtParamDistance, dIntersectionPoint] = RayTraceLaserBeam(strTargetModelData, ...
    dBeamDirection_TB, ...
    dRayOrigin_TB, ...
    dTargetPosition_TB, ...
    bEnableHeuristicPruning) %#codegen
arguments
    strTargetModelData      (1,1) struct {isstruct, isscalar}   % Struct containing mesh representing target
    dBeamDirection_TB       (3,1) double {isnumeric,isvector}   % Ray direction as unit vector
    dRayOrigin_TB           (3,1) double {isnumeric, isvector}  = [0;0;0] % Target model centre position
    dTargetPosition_TB      (3,1) double {isnumeric, isvector}  = [0;0;0] % Distance from the ray origin to the target centre
    bEnableHeuristicPruning (1,1) logical {islogical, isscalar} = false   % Flag to enable heuristic pruning for computation accel.
end
%% SIGNATURE
% [bInsersectionFlag, dtParamDistance, dIntersectionPoint] = RayTraceLaserBeam(strTargetModelData, ...
%     dBeamDirection_TB, ...
%     dRayOrigin_TB, ...
%     dTargetPosition_TB, ...
%     bEnableHeuristicPruning) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function implementing ray tracing of a single ray against mesh in strTargetModelData. Computation validity
% is checked considering the distance from ray origin to mesh centre.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strTargetModelData      (1,1) struct {isstruct, isscalar}   % Struct containing mesh representing target
% dBeamDirection_TB       (3,1) double {isnumeric,isvector}   % Ray direction as unit vector
% dRayOrigin_TB           (3,1) double {isnumeric, isvector}  = [0;0;0] % Target model centre position
% dTargetPosition_TB      (3,1) double {isnumeric, isvector}  = [0;0;0] % Distance from the ray origin to the target centre
% bEnableHeuristicPruning (1,1) logical {islogical, isscalar} = false   % Flag to enable heuristic pruning for computation accel.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dMeasDistance
% bInsersectionFlag
% bValidityFlag
% dIntersectionPoint
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 16-01-2024    Pietro Califano     First implementation for RCS-1 simulator
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------


% Check if the input structure contains the required fields
assert(isfield(strTargetModelData, 'i32triangVertexPtrs') || ...
    isfield(strTargetModelData, 'dVerticesPositions'), ...
    'Error: strTargetModelData must contain fields i32triangVertexPtrs and dVerticesPositions');

ui32NumOfTriangles = uint32(size(strTargetModelData.i32triangVertexPtrs, 2));

% if bEnableHeuristicPruning == true 
    % TODO (PC)
    % Implement heuristic pruning to reduce number of meshes to trace against
% else
    % Use full mesh model
    % TODO (PC)
    ui32NumTrianglesInSubset  = ui32NumOfTriangles;
    ui32TrianglesIDsubset     = 1:ui32NumOfTriangles;
% end


% Initialize outputs
bInsersectionFlag   = false;
dtParamDistance     = -1;
dIntersectionPoint  = zeros(3,1);

%% Intersection computations
% DEVNOTE: may be modified to select only a subset of the triangles for further optimization

% Get data from struct to avoid accessing struct mem. locations during loop
dVerticesPos        = strTargetModelData.dVerticesPositions;
i32triangVertPtr1   = strTargetModelData.i32triangVertexPtrs(1, :); % Improved mem access speed
i32triangVertPtr2   = strTargetModelData.i32triangVertexPtrs(2, :); 
i32triangVertPtr3   = strTargetModelData.i32triangVertexPtrs(3, :); 

% Compute ray origin to target centre position vector
dRayOriginToTargetCentre = dTargetPosition_TB - dRayOrigin_TB;

% Compute threshold for intersection check (projection of dRayOriginToTargetCentre onto ray)
dIntersectionValidityThr = dot(dRayOriginToTargetCentre, dBeamDirection_TB);

if dIntersectionValidityThr < 0
    % DEVNOTE: Safe exit, this should not even be reached in principle: intersection check can be skipped.
    return;
end

for id = 1:ui32NumTrianglesInSubset

    idT = ui32TrianglesIDsubset(id);
    assert(idT <= ui32NumOfTriangles)

    % Get triangle vertices ptrs
    i32triangVertPtr1_tmp = i32triangVertPtr1(id);
    i32triangVertPtr2_tmp = i32triangVertPtr2(id);
    i32triangVertPtr3_tmp = i32triangVertPtr3(id);

    % Get triangle vertices positions
    tmpTriangleVertices = coder.nullcopy(zeros(3, 3));

    tmpTriangleVertices(1:3, 1) = dVerticesPos(:, i32triangVertPtr1_tmp );
    tmpTriangleVertices(1:3, 2) = dVerticesPos(:, i32triangVertPtr2_tmp );
    tmpTriangleVertices(1:3, 3) = dVerticesPos(:, i32triangVertPtr3_tmp );

    % Ray trace to get intersection point if any
    [bTmpIntersectFlag, ~, ~, dTmptParamDistance, dTmpIntersectionPoint] = fastRayTriangleIntersection(dRayOrigin_TB, ...
                                                                                                 dBeamDirection_TB, ...
                                                                                                 tmpTriangleVertices(:, 1), ...
                                                                                                 tmpTriangleVertices(:, 2), ...
                                                                                                 tmpTriangleVertices(:, 3));
    
    % If intersection AND distance less than distance to target centre --> found, can break and return
    if bTmpIntersectFlag

        if dTmptParamDistance < dIntersectionValidityThr && dTmptParamDistance > 0
            % Intersection occurred on the camera side of the target --> VALID
            bInsersectionFlag = true;
            dtParamDistance = dTmptParamDistance;
            dIntersectionPoint = dTmpIntersectionPoint;
            return;
        end
        % Else continue, intersection if not the one to find!
        continue;
    else
        continue;
    end

end

%% LOCAL FUNCTION for portability
    function [dIsIntersected, dUbarycenCoord, dVbarycenCoord, dRangeToIntersection, dIntersectionPoint] =  fastRayTriangleIntersection( ...
            dRayOrigin, ...
            dLosToPoint, ...
            dTriangVert0, ...
            dTriangVert1, ...
            dTriangVert2)%#codegen
        % arguments
        %     dRayOrigin   (3,1)  double   {isvector, isnumeric}
        %     dLosToPoint  (3,1)  double   {isvector, isnumeric}
        %     dTriangVert0 (3,1)  double   {isvector, isnumeric}
        %     dTriangVert1 (3,1)  double   {isvector, isnumeric}
        %     dTriangVert2 (3,1)  double   {isvector, isnumeric}
        % end
        % Ray/triangle intersection using the algorithm proposed by M�ller and Trumbore (1997).
        %
        % Input:
        % dRayOrigin  : [3,1] Position vector of the ray origin
        % dLosToPoint : [3,1] Unit vector from ray origin to point
        % dTriangVert0: [3,1] Position vector of triangle vertex 0
        % dTriangVert1: [3,1] Position vector of triangle vertex 1
        % dTriangVert2: [3,1] Position vector of triangle vertex 2
        % Output:
        % dIsIntersected: (0) Reject, (1) Intersect.
        % dUbarycenCoord: X Barycentric coordinate of the intersection point
        % dVbarycenCoord: Y Barycentric coordinate of the intersection point
        % dRangeToIntersection: Distance from the ray origin to the intersection
        % dIntersectionPoint: Intersection point position in 3D reference frame (the same as triangle vertices)
        % Author:
        % Originally written by Jesus Mena, edited by David Berman (dberm22@gmail.com)
        % Improved and mex versions by Peter Califano (petercalifano.gs@gmail.com), 13-04-2024

        % INPUT ASSERT CHECKS
        assert((iscolumn(dTriangVert0) && iscolumn(dTriangVert1) && iscolumn(dTriangVert2)) && ...
            (length(dTriangVert0) == 3 && length(dTriangVert1) == 3 && length(dTriangVert2) == 3), ...
            'ERROR: input vertex positions must be [3x1] vectors!');

        assert(iscolumn(dRayOrigin) && length(dRayOrigin) == 3, ...
            'ERROR: input origin position must be [3x1] vectors!');

        epsilon = 2*eps;

        %% MAIN COMPUTATION BODY
        e1 = dTriangVert1-dTriangVert0;
        e2 = dTriangVert2-dTriangVert0;
        q = [dLosToPoint(2)*e2(3)-dLosToPoint(3)*e2(2), dLosToPoint(3)*e2(1)-dLosToPoint(1)*e2(3), dLosToPoint(1)*e2(2)-dLosToPoint(2)*e2(1)]; %cross product
        a = e1(1)*q(1)+e1(2)*q(2)+e1(3)*q(3); % determinant of the matrix M

        if (a>-epsilon && a<epsilon)
            % the vector is parallel to the plane (the intersection is at infinity)
            dIsIntersected=false;
            dUbarycenCoord=0;
            dVbarycenCoord=0;
            dRangeToIntersection=0;
            dIntersectionPoint = zeros(3,1);
            return;
        end

        f = 1/a;
        s = dRayOrigin-dTriangVert0;
        dUbarycenCoord = f*(s(1)*q(1)+s(2)*q(2)+s(3)*q(3));

        if (dUbarycenCoord<0.0)
            % the intersection is outside of the triangle
            dIsIntersected=false;
            dUbarycenCoord=0;
            dVbarycenCoord=0;
            dRangeToIntersection=0;
            dIntersectionPoint = zeros(3,1);
            return;
        end

        r = [s(2)*e1(3)-s(3)*e1(2), s(3)*e1(1)-s(1)*e1(3), s(1)*e1(2)-s(2)*e1(1)];
        dVbarycenCoord = f*(dLosToPoint(1)*r(1)+dLosToPoint(2)*r(2)+dLosToPoint(3)*r(3));

        if (dVbarycenCoord<0.0 || dUbarycenCoord+dVbarycenCoord>1.0)
            % the intersection is outside of the triangle
            dIsIntersected=false;
            dUbarycenCoord=0;
            dVbarycenCoord=0;
            dRangeToIntersection=0;
            dIntersectionPoint = zeros(3,1);
            return;
        end

        dRangeToIntersection = f*(e2(1)*r(1)+e2(2)*r(2)+e2(3)*r(3)); % verified!
        dIsIntersected = true;

        % Compute intersection point if intersection is found
        dIntersectionPoint = coder.nullcopy(zeros(3,1));
        dIntersectionPoint(:,1) = (1 - dUbarycenCoord - dVbarycenCoord) * dTriangVert0 + dUbarycenCoord * dTriangVert1 + dVbarycenCoord * dTriangVert2;

    end

end
