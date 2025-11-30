function [bInsersectionFlag, dtParamDistance, dIntersectionPoint] = RayTraceTriangMesh(strTargetModelData, ...
    dRayDirection_TB, ...
    dRayOrigin_TB, ...
    dTargetPosition_TB, ...
    bEnableHeuristicPruning) %#codegen
arguments
    strTargetModelData      (1,1) struct {isstruct, isscalar}   % Struct containing mesh representing target
    dRayDirection_TB       (3,1) double {isnumeric,isvector}   % Ray direction as unit vector
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
% 16-04-2024    Pietro Califano     Upgrade with improved version of ray-triangle intersection
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
dIntersectionValidityThr = dot(dRayOriginToTargetCentre, dRayDirection_TB);

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
    [bTmpIntersectFlag, ~, ~, dTmptParamDistance, dTmpIntersectionPoint] = RayTriangleIntersection_MollerTrumbore(dRayOrigin_TB, ...
                                                                                                                     dRayDirection_TB, ...
                                                                                                                     tmpTriangleVertices(:, 1), ...
                                                                                                                     tmpTriangleVertices(:, 2), ...
                                                                                                                     tmpTriangleVertices(:, 3), ...
                                                                                                                     true, ...
                                                                                                                     false);
    
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

%% LOCAL FUNCTION FOR PORTABILITY
    function [bIntersectionFlag, dUbarycenCoord, dVbarycenCoord, ...
            dtRangeToIntersection, dIntersectionPoint] =  RayTriangleIntersection_MollerTrumbore(dRayOrigin, ...
                                                                                                dRayDirection, ...
                                                                                                dTriangVert0, ...
                                                                                                dTriangVert1, ...
                                                                                                dTriangVert2, ...
                                                                                                bTwoSidedTest, ...
                                                                                                bCheckOcclusionOnly) %#codegen
        % arguments
        %     dRayOrigin          %(3,1)  double   % {isvector, isnumeric} % Removed for speed up. Enable is debug.
        %     dRayDirection       %(3,1)  double   % {isvector, isnumeric} %
        %     dTriangVert0        %(3,1)  double   % {isvector, isnumeric} %
        %     dTriangVert1        %(3,1)  double   % {isvector, isnumeric} %
        %     dTriangVert2        %(3,1)  double   % {isvector, isnumeric} %
        %     bTwoSidedTest       %(1,1)  logical = true;
        %     bCheckOcclusionOnly %(1,1)  logical = false; % Ray is of shadow type
        % end
        %% SIGNATURE
        % [bIntersectionFlag, dUbarycenCoord, dVbarycenCoord, ...
        %     dtRangeToIntersection, dIntersectionPoint] =  RayTriangleIntersection_MollerTrumbore( ...
        %                                                                               dRayOrigin, ...
        %                                                                               dRayDirection, ...
        %                                                                               dTriangVert0, ...
        %                                                                               dTriangVert1, ...
        %                                                                               dTriangVert2, ...
        %                                                                               bTwoSidedTest, ...
        %                                                                               bCheckOcclusionOnly) %#codegen
        % -------------------------------------------------------------------------------------------------------------
        %% DESCRIPTION
        % Implementation of Ray-triangle intersection test algorithm by Moller and Trumbore, original version
        % published in 1997. Reference: https://doi.org/10.1145/1198555.1198746
        % Additional optimizations varying the order of operations were later shown by Moller, Haines in blog post:
        % https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/raytri/. This code does not implement them.
        % -------------------------------------------------------------------------------------------------------------
        %% INPUT
        % dRayOrigin            (3,1)  double   {isvector, isnumeric} Position vector of the ray origin
        % dRayDirection         (3,1)  double   {isvector, isnumeric} Unit vector from ray origin to point
        % dTriangVert0          (3,1)  double   {isvector, isnumeric} Position vector of triangle vertex 0
        % dTriangVert1          (3,1)  double   {isvector, isnumeric} Position vector of triangle vertex 1
        % dTriangVert2          (3,1)  double   {isvector, isnumeric} Position vector of triangle vertex 2
        % bTwoSidedTest         (1,1) logical = true;  % Perform full two-sided test (back-facing triang not culled)
        % bCheckOcclusionOnly   (1,1) logical = false; % Ray is of shadow type
        % -------------------------------------------------------------------------------------------------------------
        %% OUTPUT
        % bIntersectionFlag:    (0) Miss, (1) Hit.
        % dUbarycenCoord:       X Barycentric coordinate of the intersection point
        % dVbarycenCoord:       Y Barycentric coordinate of the intersection point
        % dRangeToIntersection: Distance from the ray origin to the intersection
        % dIntersectionPoint:   Intersection point position in 3D reference frame (the same as triangle vertices)
        % -------------------------------------------------------------------------------------------------------------
        %% CHANGELOG
        % 03-02-2025    Pietro Califano     Implemented from original paper (Moore-Trumbore, 1997) with shadow rays
        % -------------------------------------------------------------------------------------------------------------
        %% DEPENDENCIES
        % [-]
        % -------------------------------------------------------------------------------------------------------------
        %% Future upgrades
        % [-]
        % -------------------------------------------------------------------------------------------------------------

        % INPUT ASSERT CHECKS
        assert((iscolumn(dTriangVert0) && iscolumn(dTriangVert1) && iscolumn(dTriangVert2)) && ...
            (length(dTriangVert0) == 3 && length(dTriangVert1) == 3 && length(dTriangVert2) == 3), ...
            'ERROR: input vertex positions must be [3x1] vectors!');

        assert(iscolumn(dRayOrigin) && length(dRayOrigin) == 3, ...
            'ERROR: input origin position must be [3x1] vectors!');

        assert( all(abs(dRayDirection) <= 1), 'Ray direction is not a unit vector!');

        % Determine machine precision to use
        EPS = eps('double');

        %% MAIN COMPUTATION BODY

        % Compute triangle edges
        dEdge1 = dTriangVert1 - dTriangVert0;
        dEdge2 = dTriangVert2 - dTriangVert0;

        % Compute auxiliary P = cross(D, E2) where D is the ray direction;
        % dP = [dRayDirection(2)*dEdge2(3) - dRayDirection(3)*dEdge2(2), ...
        %      dRayDirection(3)*dEdge2(1) - dRayDirection(1)*dEdge2(3), ...
        %      dRayDirection(1)*dEdge2(2) - dRayDirection(2)*dEdge2(1)];

        dP = cross(dRayDirection, dEdge2);
        % Compute determinant of linear system
        dDet = dot(dEdge1, dP);
        % dDet = dEdge1(1)*dP(1) + dEdge1(2)*dP(2) + dEdge1(3)*dP(3);

        % Determine condition to check based on test type
        if bTwoSidedTest
            %% Two-sided test
            % Check if ray is parallel to the plane (the intersection is at infinity)
            if (dDet > -EPS && dDet < EPS)

                bIntersectionFlag       = false;
                dUbarycenCoord          = 0;
                dVbarycenCoord          = 0;
                dtRangeToIntersection   = 0;
                dIntersectionPoint      = zeros(3,1);
                return;
            end

            % Compute inverse determinant to get "scale"
            dInvDet = 1.0/dDet;

            % Compute u barycentric coordinate
            dRayOriginFromV0 = dRayOrigin - dTriangVert0;
            dUbarycenCoord = dInvDet * dot(dRayOriginFromV0, dP);
            % dUbarycenCoord = dInvDet * (dRayOriginFromV0(1)*dP(1) + dRayOriginFromV0(2)*dP(2) + dRayOriginFromV0(3)*dP(3));

            if (dUbarycenCoord < 0.0)
                % Triangle u-missed
                bIntersectionFlag       = false;
                dUbarycenCoord          = 0;
                dVbarycenCoord          = 0;
                dtRangeToIntersection   = 0;
                dIntersectionPoint      = zeros(3,1);
                return;
            end

            % Compute auxiliary Q = cross(T, E1) where T = O - V0, and O is the ray origin;
            % T = dRayOriginFromV0 = dRayOrigin - dTriangVert0 here.
            dQ = cross(dRayOriginFromV0, dEdge1);
            % dQ = [dRayOriginFromV0(2)*dEdge1(3) - dRayOriginFromV0(3)*dEdge1(2), ...
            %       dRayOriginFromV0(3)*dEdge1(1) - dRayOriginFromV0(1)*dEdge1(3), ...
            %       dRayOriginFromV0(1)*dEdge1(2) - dRayOriginFromV0(2)*dEdge1(1)];

            % Compute V barycentric coordinate
            % dVbarycenCoord = dInvDet*(dRayDirection(1)*dQ(1) + dRayDirection(2)*dQ(2) + dRayDirection(3)*dQ(3));
            dVbarycenCoord = dInvDet * dot(dRayDirection, dQ);

            if (dVbarycenCoord < 0.0 || dUbarycenCoord + dVbarycenCoord > 1.0)
                % Triangle v-missed
                bIntersectionFlag       = false;
                dUbarycenCoord          = 0;
                dVbarycenCoord          = 0;
                dtRangeToIntersection   = 0;
                dIntersectionPoint      = zeros(3,1);
                return;
            end

            % Both (u,v) valid --> Intersection exists
            bIntersectionFlag = true;
            % Compute t parameter of intersection point in unscaled space
            dtRangeToIntersection = dInvDet * dot(dEdge2, dQ);

        else
            %% One-sided test
            % Check if ray is parallel to the plane and if can be discarded because back-facing (one-sided test)
            if dDet < EPS
                bIntersectionFlag       = false;
                dUbarycenCoord          = 0;
                dVbarycenCoord          = 0;
                dtRangeToIntersection   = 0;
                dIntersectionPoint      = zeros(3,1);
                return
            end

            % Compute T vector
            dRayOriginFromV0 = dRayOrigin - dTriangVert0;

            % Compute scaled U barycentric coordinate
            dUbarycenCoord = dot(dRayOriginFromV0, dP);

            if (dUbarycenCoord < 0.0 && dUbarycenCoord > dDet)
                % Triangle u-missed
                bIntersectionFlag       = false;
                dUbarycenCoord          = 0;
                dVbarycenCoord          = 0;
                dtRangeToIntersection   = 0;
                dIntersectionPoint      = zeros(3,1);
                return;
            end

            % Compute auxiliary Q = cross(T, E1) where T = O - V0, and O is the ray origin;
            % T = dRayOriginFromV0 = dRayOrigin - dTriangVert0 here.
            dQ = cross(dRayOriginFromV0, dEdge1);

            % [dRayOriginFromV0(2)*dEdge1(3) - dRayOriginFromV0(3)*dEdge1(2), ...
            %       dRayOriginFromV0(3)*dEdge1(1) - dRayOriginFromV0(1)*dEdge1(3), ...
            %       dRayOriginFromV0(1)*dEdge1(2) - dRayOriginFromV0(2)*dEdge1(1)];

            % Compute scaled V barycentric coordinate
            % dVbarycenCoord = (dRayDirection(1)*dQ(1) + dRayDirection(2)*dQ(2) + dRayDirection(3)*dQ(3));
            dVbarycenCoord = dot(dRayDirection, dQ);


            if (dVbarycenCoord < 0.0 || dUbarycenCoord + dVbarycenCoord > dDet)
                % Triangle v-missed
                bIntersectionFlag       = false;
                dUbarycenCoord          = 0;
                dVbarycenCoord          = 0;
                dtRangeToIntersection   = 0;
                dIntersectionPoint      = zeros(3,1);
                return;
            end

            % Both (u,v) valid --> Intersection exists
            bIntersectionFlag = true;

            % Compute auxiliary quantity before division, for shadow rays
            dEdge2dotQ = dot(dEdge2, dQ);

            % Check occlusion only case (do not compute t param and intersection point) % For shadow rays
            if bCheckOcclusionOnly && ( (dEdge2dotQ > 0 && dDet < 0 ) || (dEdge2dotQ < 0 && dDet > 0 ) )
                dUbarycenCoord          = 0; % Reset to zero, because (U,V) here are not in the unscaled space
                dVbarycenCoord          = 0;
                dtRangeToIntersection   = -1;
                dIntersectionPoint      = zeros(3,1);
                return;
            end

            % Compute t parameter of intersection point in unscaled space
            dInvDet = 1./dDet;
            dtRangeToIntersection = dInvDet * dEdge2dotQ;

            % Scale (U,V) barycentric coordinates
            dUbarycenCoord = dInvDet * dUbarycenCoord;
            dVbarycenCoord = dInvDet * dVbarycenCoord;

        end % End of intersection algorithm

        % Recover intersection point if required
        % Compute intersection point if intersection is found
        dIntersectionPoint = zeros(3,1);

        if nargout > 4 && bIntersectionFlag
            dIntersectionPoint(1:3) = (1 - dUbarycenCoord - dVbarycenCoord) * dTriangVert0 + ...
                dUbarycenCoord * dTriangVert1 + ...
                dVbarycenCoord * dTriangVert2;
        end

    end


end
