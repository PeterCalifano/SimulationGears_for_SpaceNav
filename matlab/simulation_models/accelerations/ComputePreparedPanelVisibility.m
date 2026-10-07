function dVisibleFraction = ComputePreparedPanelVisibility(dSunDirection, dPanelNormals, strShadowData) %#codegen
%% SIGNATURE
% dVisibleFraction = ComputePreparedPanelVisibility(dSunDirection, dPanelNormals, strShadowData)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate equal-area self-shadowing using prepared triangle geometry.
% Cache the common Sun direction once for all parallel sample rays. Keep opaque
% blockers two-sided and exclude the emitting source triangle. Validate geometry
% in host preparation; retain fixed-size runtime data for compiled evaluation.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSunDirection    (3,1) spacecraft-to-Sun direction in the mesh frame [-].
% dPanelNormals    (3,N) illuminated-side normals [-].
% strShadowData    Samples, ray offset and strRayData in one length unit.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVisibleFraction (N,1) visible fraction; zero for back-facing faces [-].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% TraceTriangleRay.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dSunDirection (3, 1) double
    dPanelNormals (3, :) double
    strShadowData (1, 1) struct
end

arguments (Output)
    dVisibleFraction (:, 1) double
end

% Normalize once and cache the direction coefficients for every blocker.
dDirectionNorm = norm(dSunDirection);
assert(dDirectionNorm > eps, 'ComputePanelSunVisibility:ZeroSunDirection', ...
    'Sun direction must be nonzero.');

dSunDirection = dSunDirection / dDirectionNorm;
ui32FaceCount = uint32(size(dPanelNormals, 2));
dVisibleFraction = zeros(double(ui32FaceCount), 1);

if ui32FaceCount == 0
    return
end

strRayData = strShadowData.strRayData;
dEdge2 = strRayData.dEdge2;

dCrossEdge2 = [ ...
    dSunDirection(2) * dEdge2(3, :) - dSunDirection(3) * dEdge2(2, :); ...
    dSunDirection(3) * dEdge2(1, :) - dSunDirection(1) * dEdge2(3, :); ...
    dSunDirection(1) * dEdge2(2, :) - dSunDirection(2) * dEdge2(1, :)];
dDet = sum(strRayData.dEdge1 .* dCrossEdge2, 1);
dInverseDet = zeros(1, size(dDet, 2));

for ui32Triangle = uint32(1):strRayData.ui32TriangleCount
    % Preserve the original shadow primitive's determinant tolerance.
    if abs(dDet(ui32Triangle)) >= 2 * eps
        dInverseDet(ui32Triangle) = 1 / dDet(ui32Triangle);
    end
end
strQuery = struct('dDirection', dSunDirection, 'bAnyHit', true, ...
    'bTwoSided', true, 'dMinDistance', strShadowData.dRayOffset, ...
    'dMaxDistance', Inf, 'ui32IgnoreTriangle', uint32(0), ...
    'dCrossEdge2', dCrossEdge2, 'dInverseDet', dInverseDet);

% Bound each triangle in a Sun-parallel projection before intersection work.
[~, dDominantAxis] = max(abs(dSunDirection));
ui8Axes = uint8([mod(dDominantAxis, 3)+1, mod(dDominantAxis+1, 3)+1]);
dShear = dSunDirection(ui8Axes) / dSunDirection(dDominantAxis);

dProjected0 = strRayData.dVertex0(ui8Axes, :) - ...
    dShear .* strRayData.dVertex0(dDominantAxis, :);
dProjected1 = dProjected0 + strRayData.dEdge1(ui8Axes, :) - ...
    dShear .* strRayData.dEdge1(dDominantAxis, :);
dProjected2 = dProjected0 + strRayData.dEdge2(ui8Axes, :) - ...
    dShear .* strRayData.dEdge2(dDominantAxis, :);
dProjectionPadding = 64 * eps * max(1, max(abs(strRayData.dVertex0), [], 'all') + ...
    max(abs(strRayData.dEdge1), [], 'all') + max(abs(strRayData.dEdge2), [], 'all'));

strQuery.ui8ProjectionAxes = ui8Axes;
strQuery.ui8DominantAxis = uint8(dDominantAxis);
strQuery.dProjectionShear = dShear;
strQuery.dProjectedMin = min(min(dProjected0, dProjected1), dProjected2) - dProjectionPadding;
strQuery.dProjectedMax = max(max(dProjected0, dProjected1), dProjected2) + dProjectionPadding;
strQuery.ui32CandidateTriangles = zeros(1, size(strRayData.dVertex0, 2), 'uint32');
strQuery.ui32CandidateCount = uint32(0);

% Count visible samples while retaining every face as an opaque blocker.
ui32SampleCount = uint32(size(strShadowData.dSamplePoints_SCB, 2));

for ui32Face = uint32(1):ui32FaceCount
    if dot(dPanelNormals(:, ui32Face), dSunDirection) <= 0
        continue
    end

    strQuery.ui32IgnoreTriangle = ui32Face;
    dOrigins = strShadowData.dSamplePoints_SCB(:, :, ui32Face) + strShadowData.dRayOffset * dSunDirection;

    if ~strRayData.bUseBvh

        % Reject blockers disjoint from the complete emitter sample bundle.
        % Reuse this conservative list for every parallel sample ray.
        dProjectedOrigins = dOrigins(ui8Axes, :) - dShear .* dOrigins(dDominantAxis, :);
        dEmitterMin = min(dProjectedOrigins, [], 2);
        dEmitterMax = max(dProjectedOrigins, [], 2);
        strQuery.ui32CandidateCount = uint32(0);

        for ui32Triangle = uint32(1):strRayData.ui32TriangleCount
            if ui32Triangle ~= ui32Face && dInverseDet(ui32Triangle) ~= 0 && ...
                    all(dEmitterMax >= strQuery.dProjectedMin(:, ui32Triangle)) && ...
                    all(dEmitterMin <= strQuery.dProjectedMax(:, ui32Triangle))

                strQuery.ui32CandidateCount = strQuery.ui32CandidateCount + 1;
                strQuery.ui32CandidateTriangles(strQuery.ui32CandidateCount) = ui32Triangle;

            end
        end
        if strQuery.ui32CandidateCount == 0
            % Accept the whole bundle when no opaque blocker can intersect it.
            dVisibleFraction(ui32Face) = 1;
            continue
        end
    end

    ui32VisibleCount = uint32(0);

    for ui32Sample = uint32(1):ui32SampleCount

        dOrigin = dOrigins(:, ui32Sample);
        bBlocked = TraceTriangleRay(strRayData, dOrigin, strQuery);
        if ~bBlocked
            ui32VisibleCount = ui32VisibleCount + 1;
        end
    end

    dVisibleFraction(ui32Face) = double(ui32VisibleCount) / double(ui32SampleCount);
end
end
