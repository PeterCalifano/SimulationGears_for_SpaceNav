function testPreparedTriangleRay()
%% SIGNATURE
% testPreparedTriangleRay()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Verify nearest positive hit, source identity, query intervals and misses for
% flat and multi-leaf BVH traversal using analytically placed parallel triangles.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; assert the ray-query contract for both traversal modes.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 08-10-2026  Pietro Califano  Verify the imported tracing API during SRP harmonization.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildTriangleRayData, ValidateTriangleRayData, TraceTriangleRay.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
end

% Put the closest triangle late in source order and force multiple BVH leaves.
dTriangle = [0,1,0;0,0,1;0,0,0];
dHeights = [9:-1:1,1];
dVertices = repmat(dTriangle,1,1,numel(dHeights));
dVertices(3,:,:) = repmat(reshape(dHeights,1,1,[]),1,3,1);
strQuery = struct('dDirection',[0;0;1],'bAnyHit',false,'bTwoSided',true, ...
    'dMinDistance',0,'dMaxDistance',Inf,'ui32IgnoreTriangle',uint32(0));
dOrigin = [0.2;0.2;0];
for bUseBvh = [false,true]
    strRayData = BuildTriangleRayData(dVertices,bUseBvh);
    ValidateTriangleRayData(strRayData);
    [bHit,dDistance,dPoint,ui32Id] = TraceTriangleRay(strRayData,dOrigin,strQuery);
    assert(bHit && dDistance==1 && ui32Id==9 && isequal(dPoint,[0.2;0.2;1]));

    % Ignore one tied source, then test the open lower and closed upper bounds.
    strChanged = strQuery;
    strChanged.ui32IgnoreTriangle = uint32(9);
    [bHit,dDistance,~,ui32Id] = TraceTriangleRay(strRayData,dOrigin,strChanged);
    assert(bHit && dDistance==1 && ui32Id==10);
    strChanged = strQuery;
    strChanged.dMinDistance = 1;
    strChanged.dMaxDistance = 2;
    [bHit,dDistance,~,ui32Id] = TraceTriangleRay(strRayData,dOrigin,strChanged);
    assert(bHit && dDistance==2 && ui32Id==8);

    % Ray parameters scale inversely with direction magnitude; geometry does not.
    strChanged = strQuery;
    strChanged.dDirection = [0;0;2];
    [bHit,dDistance,dPoint] = TraceTriangleRay(strRayData,dOrigin,strChanged);
    assert(bHit && dDistance==0.5 && isequal(dPoint,[0.2;0.2;1]));
    strChanged.bAnyHit = true;
    assert(TraceTriangleRay(strRayData,dOrigin,strChanged));

    % Triangles behind the origin and rays outside their bounds must miss.
    strChanged = strQuery;
    strChanged.dDirection = [0;0;-1];
    [bHit,dDistance,dPoint,ui32Id] = TraceTriangleRay(strRayData,dOrigin,strChanged);
    assert(~bHit && dDistance==-1 && ui32Id==0 && isequal(dPoint,zeros(3,1)));
    assert(~TraceTriangleRay(strRayData,[2;2;0],strQuery));
end
fprintf('Prepared triangle flat/BVH nearest-hit contracts passed.\n');
end
