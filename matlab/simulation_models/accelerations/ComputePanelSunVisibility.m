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
% 06-10-2026  Codex (GPT-6)  Reuse vectorized prepared parallel-ray visibility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePreparedPanelVisibility.
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

% Enforce one shared triangle index and at least one quadrature sample per face.
ui32FaceCount = uint32(size(dPanelNormals_SCB, 2));
ui32SampleCount = uint32(size(dSamplePoints_SCB, 2));
assert(size(dSamplePoints_SCB, 3) == ui32FaceCount && ...
       size(dFaceVertices_SCB, 3) == ui32FaceCount && ui32SampleCount > 0, ...
       'ComputePanelSunVisibility:GeometrySizeMismatch', ...
       'Normals, samples and vertices must share triangle indices and nonempty quadrature.');
% Share the prepared numerical kernel with full-grid MATLAB/MEX construction.
strShadow = struct('dSamplePoints_SCB', dSamplePoints_SCB, ...
    'dFaceVertices_SCB', dFaceVertices_SCB, 'dRayOffset', dRayOffset);
dVisibleFraction = ComputePreparedPanelVisibility(dSunDir_SCB, dPanelNormals_SCB, strShadow);
end
