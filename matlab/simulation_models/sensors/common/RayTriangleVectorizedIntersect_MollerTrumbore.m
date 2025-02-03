function [bIsIntersected, dUbarycenCoord, dVbarycenCoord, ...
    dRangeToIntersection, dIntersectionPoint] = RayTriangleVectorizedIntersect_MollerTrumbore(dRayOrigin, ...
                                                                                             dRayDirection, ...
                                                                                             dAllTriangVert0, ...
                                                                                             dAllTriangVert1, ...
                                                                                             dAllTriangVert2, ...
                                                                                             ui32NumOfTriang, ...
                                                                                             bTwoSidedTest)%#codegen
arguments
    dRayOrigin          double %(3,1)  double   {isvector, isnumeric} % Removed for speed up. Enable is debug.
    dRayDirection       double %(3,1)  double   {isvector, isnumeric} % 
    dAllTriangVert0     double %(3,:)  double   {isvector, isnumeric} % 
    dAllTriangVert1     double %(3,:)  double   {isvector, isnumeric} % 
    dAllTriangVert2     double %(3,:)  double   {isvector, isnumeric} % 
    ui32NumOfTriang     uint32
    bTwoSidedTest       (1,1) logical 
end

% Precompute edges once
persistent dTriangEdges1 dTriangEdges2

if isempty(dTriangEdges1)
    dTriangEdges1 = dAllTriangVert1 - dAllTriangVert0;
end

if isempty(dTriangEdges2)
    dTriangEdges2 = dAllTriangVert2 - dAllTriangVert0;
end

% Initialize outputs
bIsIntersected          = false; 
dUbarycenCoord          = 0.0;
dVbarycenCoord          = 0.0;
dRangeToIntersection    = 0.0;
dIntersectionPoint      = zeros(3,1);

% Expand ray direction to size of mesh
dRayDirectionArray = repmat(dRayDirection, 1, ui32NumOfTriang);  % [3 x Ntri]

% Compute P vectors array
dParray = cross(dRayDirectionArray, dTriangEdges2);

% Compute Determinant
dDetArray = sum(dTriangEdges1 .* dParray, 1);

EPS = eps('double');

bIntersectionMask = false(1, ui32NumOfTriang);

if bTwoSidedTest
    %% Two-sided test
    % Check validity of determinant (stop computation for all triangles not passing this)
    bIntersectionMask(:) = abs(dDetArray) > EPS; 

    % Compute T vectors array
    dRayOriginFromV0array = repmat(dRayOrigin, 1, ui32NumOfTriang) - dAllTriangVert0;  % [3 x Ntri]
    
    % Compute u barycentric coordinates
    dUbarycenCoordArray = sum(dRayOriginFromV0array .* dParray, 1) ./ dDetArray;
    
    % Update intersection validity mask
    bIntersectionMask = bIntersectionMask & ...
        ( (dUbarycenCoordArray >= 0) & (dUbarycenCoordArray <= 1) );

    % Compute Q vector arrays
    dQarray = cross(dRayOriginFromV0array, dTriangEdges1);

    % Determine v barycentric coordinates
    dVbarycenCoordArray = sum(dRayDirectionArray .* dQarray, 1) ./ dDetArray;
        
    % Update intersection validity mask
    bIntersectionMask = bIntersectionMask & ...
        ( (dVbarycenCoordArray >= 0) & (dVbarycenCoordArray + dUbarycenCoordArray <= 1) );

    % Compute intersection distance
    dtRangeToIntersectionArray = sum(dTriangEdges2 .* dQarray, 1) ./ dDetArray;

    % Update intersection validity mask
    bIntersectionMask = bIntersectionMask & dtRangeToIntersectionArray >= EPS;

else
return;
end

% Find closer intersection among valid ones
bIsIntersected = any(bIntersectionMask);

% Find closest intersection point (min distance in dtRangeToIntersectionArray)
[dRangeToIntersection, dMinPos] = min(dtRangeToIntersectionArray(bIntersectionMask));

dUbarycenCoord = dUbarycenCoordArray(:, bIntersectionMask);
dVbarycenCoord = dVbarycenCoordArray(:, bIntersectionMask);

dUbarycenCoord = dUbarycenCoord(:, dMinPos);
dVbarycenCoord = dVbarycenCoord(:, dMinPos);


% Compute corresponding intersection point
dAllTriangVert0 = dAllTriangVert0(:, bIntersectionMask);
dAllTriangVert1 = dAllTriangVert1(:, bIntersectionMask);
dAllTriangVert2 = dAllTriangVert2(:, bIntersectionMask);

dIntersectionPoint = (1 - dUbarycenCoord - dVbarycenCoord) * dAllTriangVert0(:, dMinPos) + ...
                                            dUbarycenCoord * dAllTriangVert1(:, dMinPos) + ...
                                            dVbarycenCoord * dAllTriangVert2(:, dMinPos);

end




