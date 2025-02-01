function [bIntersectionFlag, dUbarycenCoord, dVbarycenCoord, ...
    dtRangeToIntersection, dIntersectionPoint] =  RayTriangleIntersection_MooreTrumbore( ...
                            dRayOrigin, ...
                            dRayDirection, ...
                            dTriangVert0, ...
                            dTriangVert1, ...
                            dTriangVert2, ...
                            bTwoSidedTest, ...
                            bCheckOcclusionOnly)%#codegen
arguments
    dRayOrigin          (3,1)  double   % {isvector, isnumeric} % Removed for speed up. Enable is debug.
    dRayDirection       (3,1)  double   % {isvector, isnumeric} % 
    dTriangVert0        (3,1)  double   % {isvector, isnumeric} % 
    dTriangVert1        (3,1)  double   % {isvector, isnumeric} % 
    dTriangVert2        (3,1)  double   % {isvector, isnumeric} % 
    bTwoSidedTest       (1,1) logical = true;
    bCheckOcclusionOnly (1,1) logical = false;
end
% Ray/triangle intersection using the algorithm proposed by M�ller and Trumbore (1997).
%
% Input:
% dRayOrigin    : [3,1] Position vector of the ray origin
% dRayDirection : [3,1] Unit vector from ray origin to point
% dTriangVert0  : [3,:] Position vector of triangle vertex 0
% dTriangVert1  : [3,:] Position vector of triangle vertex 1
% dTriangVert2  : [3,:] Position vector of triangle vertex 2
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

% Determine machine precision to use
EPS = eps('single');

%% MAIN COMPUTATION BODY
% DEVNOTE: Linear system to solve for intersection
% TO DO

% Compute triangle edges
dEdge1 = dTriangVert1 - dTriangVert0;
dEdge2 = dTriangVert2 - dTriangVert0;

% Compute auxiliary Q = cross(T, E1) where T = Origin - V0;
dQ = [dRayDirection(2)*dEdge2(3) - dRayDirection(3)*dEdge2(2), ...
    dRayDirection(3)*dEdge2(1) - dRayDirection(1)*dEdge2(3), ...
    dRayDirection(1)*dEdge2(2) - dRayDirection(2)*dEdge2(1)];  

% Compute determinant of linear system
dDet = dEdge1(1)*dQ(1) + dEdge1(2)*dQ(2) + dEdge1(3)*dQ(3); 

% Determine condition to check based on test type
if bTwoSidedTest
    % Check if ray is parallel to the plane (the intersection is at infinity)
    if (dDet > -EPS && dDet < EPS)
        
        bIntersectionFlag       = false;
        dUbarycenCoord          = 0;
        dVbarycenCoord          = 0;
        dtRangeToIntersection   = 0;
        dIntersectionPoint      = zeros(3,1);
        return;
    end

else
    % Check if ray is parallel to the plane and if can be discarded because back-facing (one-sided test)
    if dDet < EPS
        bIntersectionFlag       = false;
        dUbarycenCoord          = 0;
        dVbarycenCoord          = 0;
        dtRangeToIntersection   = 0;
        dIntersectionPoint      = zeros(3,1);
        return
    end
end


% TODO understand differences: why division is here before computing u?
% NOTE: the only division is here --> no need to get up to this point for shadow rays!
f = 1/dDet;
s = dRayOrigin-dTriangVert0;
dUbarycenCoord = f*(s(1)*dQ(1)+s(2)*dQ(2)+s(3)*dQ(3));

if (dUbarycenCoord<0.0)
    % the intersection is outside of the triangle
    bIntersectionFlag=false;
    dUbarycenCoord=0;
    dVbarycenCoord=0;
    dtRangeToIntersection=0;
    dIntersectionPoint = zeros(3,1);
    return;
end

r = [s(2)*dEdge1(3)-s(3)*dEdge1(2), s(3)*dEdge1(1)-s(1)*dEdge1(3), s(1)*dEdge1(2)-s(2)*dEdge1(1)];
dVbarycenCoord = f*(dRayDirection(1)*r(1)+dRayDirection(2)*r(2)+dRayDirection(3)*r(3));

if (dVbarycenCoord<0.0 || dUbarycenCoord+dVbarycenCoord>1.0)
    % the intersection is outside of the triangle
    bIntersectionFlag=false;
    dUbarycenCoord=0;
    dVbarycenCoord=0;
    dtRangeToIntersection=0;
    dIntersectionPoint = zeros(3,1);
    return;
end

dtRangeToIntersection = f*(dEdge2(1)*r(1)+dEdge2(2)*r(2)+dEdge2(3)*r(3)); % verified!
bIntersectionFlag = true;

% Compute intersection point if intersection is found
dIntersectionPoint = coder.nullcopy(zeros(3,1));
dIntersectionPoint(:,1) = (1 - dUbarycenCoord - dVbarycenCoord) * dTriangVert0 + dUbarycenCoord * dTriangVert1 + dVbarycenCoord * dTriangVert2;

end
