function [dIsIntersected, dUbarycenCoord, dVbarycenCoord, ...
    dRangeToIntersection, dIntersectionPoint] = RayTriangleVectorizedIntersect_MooreTrumbore(dRayOrigin, ...
                                                                                             dRayDirection, ...
                                                                                             dTriangVert0, ...
                                                                                             dTriangVert1, ...
                                                                                             dTriangVert2, ...
                                                                                             ui32NumOfTriang)%#codegen
arguments
    dRayOrigin      double %(3,1)  double   {isvector, isnumeric} % Removed for speed up. Enable is debug.
    dRayDirection   double %(3,1)  double   {isvector, isnumeric} % 
    dTriangVert0    double %(3,:)  double   {isvector, isnumeric} % 
    dTriangVert1    double %(3,:)  double   {isvector, isnumeric} % 
    dTriangVert2    double %(3,:)  double   {isvector, isnumeric} % 
    ui32NumOfTriang uint32
end

% Precompute edges once
persistent triangEdges1 triangEdges2

if isempty(triangEdges1)
triangEdges1
end

if isempty(triangEdges2)
triangEdges2
end

% Initialize outputs
dIsIntersected          = false; 
dUbarycenCoord          = 0.0;
dVbarycenCoord          = 0.0;
dRangeToIntersection    = 0.0;
dIntersectionPoint      = zeros(3,1);

% TODO (PC)


end




