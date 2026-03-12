function [bIsIntersected, dUbarycenCoord, dVbarycenCoord, ...
    dRangeToIntersection, dIntersectionPoint] = RayTriangleVectorizedIntersectWithReject_MollerTrumbore(dRayOrigin, ...
                                                                                             dRayDirection, ...
                                                                                             dAllTriangVert0, ...
                                                                                             dAllTriangVert1, ...
                                                                                             dAllTriangVert2, ...
                                                                                             ui32NumOfTriang, ...
                                                                                             bTwoSidedTest)%#codegen
arguments
    dRayOrigin          %double %(3,1)  double   {isvector, isnumeric} % Removed for speed up. Enable is debug.
    dRayDirection       %double %(3,1)  double   {isvector, isnumeric} % 
    dAllTriangVert0     %double %(3,:)  double   {isvector, isnumeric} % 
    dAllTriangVert1     %double %(3,:)  double   {isvector, isnumeric} % 
    dAllTriangVert2     %double %(3,:)  double   {isvector, isnumeric} % 
    ui32NumOfTriang     %uint32
    bTwoSidedTest       %(1,1) logical 
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
dParray = cross(dRayDirectionArray, dTriangEdges2, 1);

% Compute Determinant
dDetArray = sum(dTriangEdges1 .* dParray, 1);

EPS = eps('double');

bIntersectionMask = true(1, ui32NumOfTriang);
ui32RePointers = 1:ui32NumOfTriang;

if bTwoSidedTest
    %% Two-sided test
    % Check validity of determinant (stop computation for all triangles not passing this)
    bIntersectionMask(:) = abs(dDetArray) > EPS; % Mask over triangles!

    % TODO: attempt to add rejection to vectorized code
    ui32NumOfTriangValidDet = uint32(sum(bIntersectionMask));
    ui32RePointers(not(bIntersectionMask)) = [];

    % Compute T vectors array
    dRayOriginFromV0array = repmat(dRayOrigin, 1, ui32NumOfTriangValidDet) - dAllTriangVert0(:, bIntersectionMask);  % [3 x Ntri]
    
    % Compute u barycentric coordinates
    dUbarycenCoordArray = sum(dRayOriginFromV0array .* dParray(:, bIntersectionMask), 1) ./ dDetArray(ui32RePointers);
    
    % Update intersection validity mask
    bIntersectionMask_ValidU = ( (dUbarycenCoordArray >= 0) & (dUbarycenCoordArray <= 1) );
    
    ui32RePointers(not(bIntersectionMask_ValidU)) = [];

    ui32RePointers_Uvalid = 1:length(bIntersectionMask_ValidU);
    ui32RePointers_Uvalid(not(bIntersectionMask_ValidU)) = [];

    % Compute Q vector arrays
    dQarray = cross(dRayOriginFromV0array(:, bIntersectionMask_ValidU), dTriangEdges1(:, ui32RePointers), 1);

    % Determine v barycentric coordinates
    dVbarycenCoordArray = sum(dRayDirectionArray(:, ui32RePointers) .* dQarray, 1) ./ dDetArray(ui32RePointers);
        
    % Update intersection validity mask
    bIntersectionMask_ValidV = ( (dVbarycenCoordArray >= 0) & (dVbarycenCoordArray + dUbarycenCoordArray(bIntersectionMask_ValidU) <= 1) );
    ui32RePointers(not(bIntersectionMask_ValidV)) = [];
    
    ui32RePointers_Vvalid = 1:length(bIntersectionMask_ValidV);
    ui32RePointers_Vvalid(not(bIntersectionMask_ValidV)) = [];

    ui32RePointers_Uvalid(not(bIntersectionMask_ValidV)) = [];

    % Compute intersection distance
    dtRangeToIntersectionArray = sum(dTriangEdges2(:, ui32RePointers) .* dQarray(:, bIntersectionMask_ValidV), 1) ./ dDetArray(ui32RePointers);

    % Update intersection validity mask
    btParamValidityMask = dtRangeToIntersectionArray >= EPS;

    % bIntersectionMask(not(btParamValidityMask)) = false;
    % ui32RePointers(not(btParamValidityMask)) = [];
    % bIntersectionMask = bIntersectionMask & ;

else
return;
end

% Find closer intersection among valid ones
bIsIntersected = any(btParamValidityMask);

if bIsIntersected 
    % Find closest intersection point (min distance in dtRangeToIntersectionArray)
    [dRangeToIntersection, dMinPos] = min(dtRangeToIntersectionArray);

    dUbarycenCoord = dUbarycenCoordArray(:, ui32RePointers_Uvalid);
    dVbarycenCoord = dVbarycenCoordArray(:, ui32RePointers_Vvalid);

    dUbarycenCoord = dUbarycenCoord(:, dMinPos);
    dVbarycenCoord = dVbarycenCoord(:, dMinPos);

    % Compute corresponding intersection point
    dAllTriangVert0 = dAllTriangVert0(:, ui32RePointers);
    dAllTriangVert1 = dAllTriangVert1(:, ui32RePointers);
    dAllTriangVert2 = dAllTriangVert2(:, ui32RePointers);

    dIntersectionPoint = (1 - dUbarycenCoord - dVbarycenCoord) * dAllTriangVert0(:, dMinPos) + ...
        dUbarycenCoord * dAllTriangVert1(:, dMinPos) + ...
        dVbarycenCoord * dAllTriangVert2(:, dMinPos);
else
    dUbarycenCoord = 0.0;
    dVbarycenCoord = 0.0;
    dRangeToIntersection = 0.0;
    dIntersectionPoint = zeros(3,1);
end

end




