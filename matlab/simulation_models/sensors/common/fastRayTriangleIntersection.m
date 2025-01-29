function [dIsIntersected, dUbarycenCoord, dVbarycenCoord, dRangeToIntersection, dIntersectionPoint] =  fastRayTriangleIntersection( ...
    dRayOrigin, ...
    dLosToPoint, ...
    dTriangVert0, ...
    dTriangVert1, ...
    dTriangVert2)%#codegen
arguments
    dRayOrigin   (3,1)  double   {isvector, isnumeric}
    dLosToPoint  (3,1)  double   {isvector, isnumeric}
    dTriangVert0 (3,1)  double   {isvector, isnumeric}
    dTriangVert1 (3,1)  double   {isvector, isnumeric}
    dTriangVert2 (3,1)  double   {isvector, isnumeric}
end
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
