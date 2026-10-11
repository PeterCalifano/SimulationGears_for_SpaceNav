function dVisibleFraction = ComputePreparedPanelVisibility(dSunDirection, dPanelNormals, strShadow) %#codegen
%% SIGNATURE
% dVisibleFraction = ComputePreparedPanelVisibility(dSunDirection, dPanelNormals, strShadow)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate equal-area self-shadowing with prepared geometry. Cache intersection
% coefficients for the common Sun direction and vectorize quadrature rays against
% opaque triangles in one emitter-sized workspace. Preserve two-sided blocking,
% emitter exclusion, the original determinant tolerance, and shifted ray origins.
% Geometry validation belongs to the public visibility wrapper or LUT preparation.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSunDirection    (3,1) Spacecraft-to-Sun vector in the mesh frame.
% dPanelNormals    (3,N) Illuminated-side normals in the same frame.
% strShadow       Equal-area sample points, face vertices, ray offset, and optional
%                 strTriangleEdges or strRayData with dVertex0, dEdge1, and dEdge2.
%                 Both prepared layouts use conservative flat bundle traversal.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVisibleFraction (N,1) Directly illuminated fractions; back-facing faces are zero.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Vectorize the prepared parallel-ray visibility calculation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None; numerical Moller-Trumbore coefficients with conservative projection rejection.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dSunDirection (3,1) double
    dPanelNormals (3,:) double
    strShadow (1,1) struct
end
arguments (Output)
    dVisibleFraction (:,1) double
end

% Derive static edges only for legacy callers that do not carry prepared edges.
dDirectionNorm = norm(dSunDirection);
assert(isfinite(dDirectionNorm) && dDirectionNorm > eps, ...
    'ComputePanelSunVisibility:ZeroSunDirection', 'Sun direction must be nonzero.');
dSunDirection = dSunDirection / dDirectionNorm;
ui32FaceCount = uint32(size(dPanelNormals,2));
ui32SampleCount = uint32(size(strShadow.dSamplePoints_SCB,2));
dVisibleFraction = zeros(double(ui32FaceCount),1);
if ui32FaceCount == 0
    return
end
if isfield(strShadow,'strRayData')
    dVertex0 = strShadow.strRayData.dVertex0;
    dEdge1 = strShadow.strRayData.dEdge1;
    dEdge2 = strShadow.strRayData.dEdge2;
elseif isfield(strShadow,'strTriangleEdges')
    dVertex0 = strShadow.strTriangleEdges.dVertex0;
    dEdge1 = strShadow.strTriangleEdges.dEdge1;
    dEdge2 = strShadow.strTriangleEdges.dEdge2;
else
    dVertex0 = reshape(strShadow.dFaceVertices_SCB(:,1,:),3,[]);
    dEdge1 = reshape(strShadow.dFaceVertices_SCB(:,2,:),3,[]) - dVertex0;
    dEdge2 = reshape(strShadow.dFaceVertices_SCB(:,3,:),3,[]) - dVertex0;
end

% One determinant and inverse per blocker, shared by all parallel sample rays.
dCrossEdge2 = [dSunDirection(2)*dEdge2(3,:) - dSunDirection(3)*dEdge2(2,:); ...
    dSunDirection(3)*dEdge2(1,:) - dSunDirection(1)*dEdge2(3,:); ...
    dSunDirection(1)*dEdge2(2,:) - dSunDirection(2)*dEdge2(1,:)];
dDeterminant = sum(dEdge1 .* dCrossEdge2,1);
bNonparallel = abs(dDeterminant) >= 2*eps;
dInverseDet = double(bNonparallel) ./ (dDeterminant + double(~bNonparallel));

% Reject disjoint emitter/blocker bundles in a conservative Sun-parallel projection.
[~,dDominantAxis] = max(abs(dSunDirection));
ui32DominantAxis = uint32(dDominantAxis);
ui32Axes = [mod(ui32DominantAxis,3)+1, mod(ui32DominantAxis+1,3)+1];
dShear = dSunDirection(ui32Axes) / dSunDirection(ui32DominantAxis);
dProjected0 = dVertex0(ui32Axes,:) - dShear .* dVertex0(ui32DominantAxis,:);
dProjected1 = dProjected0 + dEdge1(ui32Axes,:) - dShear .* dEdge1(ui32DominantAxis,:);
dProjected2 = dProjected0 + dEdge2(ui32Axes,:) - dShear .* dEdge2(ui32DominantAxis,:);
dPadding = 64*eps*max(1,max(abs(dVertex0),[],'all') + ...
    max(abs(dEdge1),[],'all') + max(abs(dEdge2),[],'all'));
dProjectedMin = min(min(dProjected0,dProjected1),dProjected2) - dPadding;
dProjectedMax = max(max(dProjected0,dProjected1),dProjected2) + dPadding;
dRayShift = strShadow.dRayOffset * dSunDirection;

% Bound scratch storage by one face's samples, never by the complete sphere.
for ui32Face = uint32(1):ui32FaceCount
    if dot(dPanelNormals(:,ui32Face),dSunDirection) <= 0
        continue
    end
    dOrigins = strShadow.dSamplePoints_SCB(:,:,ui32Face) + dRayShift;
    dProjectedOrigins = dOrigins(ui32Axes,:) - dShear .* dOrigins(ui32DominantAxis,:);
    dEmitterMin = min(dProjectedOrigins,[],2);
    dEmitterMax = max(dProjectedOrigins,[],2);
    bCandidates = bNonparallel & all(dEmitterMax >= dProjectedMin,1) & ...
        all(dEmitterMin <= dProjectedMax,1);
    bCandidates(ui32Face) = false;
    if ~any(bCandidates)
        dVisibleFraction(ui32Face) = 1;
        continue
    end

    % Generated C benefits from scalar early exit; MATLAB batches the same hit law.
    if ~coder.target('MATLAB')
        ui32VisibleCount = uint32(0);
        for ui32Sample = uint32(1):ui32SampleCount
            bHit = false;
            for ui32Blocker = uint32(1):ui32FaceCount
                if ~bCandidates(ui32Blocker)
                    continue
                end
                bHit = HitPrepared_(dOrigins(:,ui32Sample),dVertex0(:,ui32Blocker), ...
                    dEdge1(:,ui32Blocker),dEdge2(:,ui32Blocker), ...
                    dCrossEdge2(:,ui32Blocker),dInverseDet(ui32Blocker), ...
                    dSunDirection,strShadow.dRayOffset);
                if bHit
                    break
                end
            end
            ui32VisibleCount = ui32VisibleCount + uint32(~bHit);
        end
        dVisibleFraction(ui32Face) = double(ui32VisibleCount)/double(ui32SampleCount);
        continue
    end
    bBlocked = any(bCandidates & HitPrepared_(dOrigins,dVertex0,dEdge1,dEdge2, ...
        dCrossEdge2,dInverseDet,dSunDirection,strShadow.dRayOffset),2);
    dVisibleFraction(ui32Face) = double(sum(~bBlocked)) / double(ui32SampleCount);
end
end

function bHit = HitPrepared_(dOrigins,dVertex0,dEdge1,dEdge2,dCrossEdge2,dInverseDet,dSunDirection,dRayOffset) %#codegen
%% SIGNATURE
% bHit = HitPrepared_(dOrigins,dVertex0,dEdge1,dEdge2,dCrossEdge2,dInverseDet,dSunDirection,dRayOffset)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Share the exact barycentric/distance hit law between MATLAB batches and
% compiled scalar early-exit queries. Do not calculate unused intersection points.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dOrigins        Shifted origins, one or several columns.
% dVertex0        Blocker origins, one or several columns.
% dEdge1, dEdge2  Prepared blocker edges.
% dCrossEdge2     Sun direction crossed with each second edge.
% dInverseDet     Nonparallel inverse determinants.
% dSunDirection  Normalized common Sun direction.
% dRayOffset     Minimum accepted distance beyond the shifted origin.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bHit           Ray-by-blocker forward-intersection mask.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 06-10-2026  Codex (GPT-6)  Specialize traversal without duplicating the numerical hit law.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dOrigins (3,:) double
    dVertex0 (3,:) double
    dEdge1 (3,:) double
    dEdge2 (3,:) double
    dCrossEdge2 (3,:) double
    dInverseDet (1,:) double
    dSunDirection (3,1) double
    dRayOffset (1,1) double
end
arguments (Output)
    bHit (:,:) logical
end
coder.inline('always');
dRelativeX = dOrigins(1,:).' - dVertex0(1,:);
dRelativeY = dOrigins(2,:).' - dVertex0(2,:);
dRelativeZ = dOrigins(3,:).' - dVertex0(3,:);
dBarycentricU = (dRelativeX.*dCrossEdge2(1,:) + ...
    dRelativeY.*dCrossEdge2(2,:) + dRelativeZ.*dCrossEdge2(3,:)).*dInverseDet;
dCrossX = dRelativeY.*dEdge1(3,:) - dRelativeZ.*dEdge1(2,:);
dCrossY = dRelativeZ.*dEdge1(1,:) - dRelativeX.*dEdge1(3,:);
dCrossZ = dRelativeX.*dEdge1(2,:) - dRelativeY.*dEdge1(1,:);
dBarycentricV = (dSunDirection(1)*dCrossX + dSunDirection(2)*dCrossY + ...
    dSunDirection(3)*dCrossZ).*dInverseDet;
dDistances = (dEdge2(1,:).*dCrossX + dEdge2(2,:).*dCrossY + ...
    dEdge2(3,:).*dCrossZ).*dInverseDet;
bHit = dBarycentricU >= 0 & dBarycentricV >= 0 & ...
    dBarycentricU+dBarycentricV <= 1 & dDistances > dRayOffset;
end
