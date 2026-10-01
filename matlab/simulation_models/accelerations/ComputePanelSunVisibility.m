function dVisibleFraction = ComputePanelSunVisibility(dSunDir_SCB, dPanelNormals_SCB, ...
    dSamplePoints_SCB, dFaceVertices_SCB, dRayOffset) %#codegen
%% SIGNATURE
% dVisibleFraction = ComputePanelSunVisibility(dSunDir_SCB, dPanelNormals_SCB, ...
%     dSamplePoints_SCB, dFaceVertices_SCB, dRayOffset)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Estimate the directly illuminated fraction of each triangle from equal-area
% samples. Cast two-sided rays toward a point-source Sun and exclude the emitting
% triangle. Shift each origin toward the Sun by dRayOffset and count a blocker
% only beyond another dRayOffset from that shifted origin. Supply all positions
% and the offset in one length unit; handle external eclipses in the caller.
% Example: dVisibleFraction = ComputePanelSunVisibility([0; 0; 1], [0; 0; 1], ...
%     [1/3; 1/3; 0], [0, 1, 0; 0, 0, 1; 0, 0, 0], 1e-8);
% Output: 1 for one unoccluded Sun-facing triangle.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSunDir_SCB         (3, 1)      Spacecraft-to-Sun direction in the mesh frame [-].
% dPanelNormals_SCB   (3, N)      Prepared illuminated-side normals in the same frame [-].
% dSamplePoints_SCB   (3, Q, N)   Equal-area quadrature points on each triangle [length].
% dFaceVertices_SCB   (3, 3, N)   Three vertices of each opaque occluding triangle [length].
% dRayOffset          (1, 1)      Positive origin shift and hit-distance tolerance [length].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVisibleFraction    (N, 1)   Sun-visible sample fraction; back-facing faces are zero [-].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-09-2026  Pietro Califano, Codex gpt-6  Add standalone panel self-shadowing geometry.
% 30-09-2026  Pietro Califano, Codex gpt-6  Clarify ray tolerance and review geometry contracts.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% RayTwoSidedTriangleIntersection_MollerTrembore.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dSunDir_SCB (3, 1) double {mustBeFinite}
    dPanelNormals_SCB (3, :) double {mustBeFinite}
    dSamplePoints_SCB (3, :, :) double {mustBeFinite}
    dFaceVertices_SCB (3, 3, :) double {mustBeFinite}
    dRayOffset (1, 1) double {mustBeFinite, mustBePositive}
end

arguments (Output)
    dVisibleFraction (:, 1) double
end

% Normalize once so ray distances share the geometry's length unit.
dSunDirNorm = norm(dSunDir_SCB);
assert(dSunDirNorm > eps, 'ComputePanelSunVisibility:ZeroSunDirection', ...
    'Sun direction must be nonzero.');
dSunDir_SCB = dSunDir_SCB / dSunDirNorm;

% Enforce one shared triangle index and at least one quadrature sample per face.
ui32FaceCount = uint32(size(dPanelNormals_SCB, 2));
ui32SampleCount = uint32(size(dSamplePoints_SCB, 2));
assert(size(dSamplePoints_SCB, 3) == ui32FaceCount && ...
       size(dFaceVertices_SCB, 3) == ui32FaceCount && ui32SampleCount > 0, ...
       'ComputePanelSunVisibility:GeometrySizeMismatch', ...
       'Normals, samples and vertices must share triangle indices and nonempty quadrature.');
dVisibleFraction = zeros(double(ui32FaceCount), 1);

% Count visible equal-area samples only on optically illuminated faces.
for ui32FaceIndex = uint32(1):ui32FaceCount
    if dot(dPanelNormals_SCB(:, ui32FaceIndex), dSunDir_SCB) <= 0
        continue
    end

    ui32VisibleCount = uint32(0);
    for ui32SampleIndex = uint32(1):ui32SampleCount
        % Stop at the first opaque blocker beyond the shifted-origin tolerance.
        dRayOrigin_SCB = dSamplePoints_SCB(:, ui32SampleIndex, ui32FaceIndex) + dRayOffset * dSunDir_SCB;
        bBlocked = false;
        for ui32BlockerIndex = uint32(1):ui32FaceCount
            if ui32BlockerIndex == ui32FaceIndex
                continue
            end

            % Run ray-triangle intersection with the Möller–Trumbore algorithm, considering two-sided triangles. Return the distance to the intersection point along the ray.
            [bHit, ~, ~, dRayHitDistance] = RayTwoSidedTriangleIntersection_MollerTrembore( ...
                dRayOrigin_SCB, dSunDir_SCB, dFaceVertices_SCB(:, 1, ui32BlockerIndex), ...
                dFaceVertices_SCB(:, 2, ui32BlockerIndex), dFaceVertices_SCB(:, 3, ui32BlockerIndex));

            if bHit && dRayHitDistance > dRayOffset
                bBlocked = true;
                break
            end
        end

        if ~bBlocked
            ui32VisibleCount = ui32VisibleCount + uint32(1);
        end
    end

    dVisibleFraction(ui32FaceIndex) = double(ui32VisibleCount) / double(ui32SampleCount);
end
end
