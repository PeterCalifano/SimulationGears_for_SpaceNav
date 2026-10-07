function dVisibleFraction = ComputePanelSunVisibility(dSunDir_SCB, dPanelNormals_SCB, ...
    dSamplePoints_SCB, dFaceVertices_SCB, dRayOffset, strRayData) %#codegen
%% SIGNATURE
% dVisibleFraction = ComputePanelSunVisibility(dSunDir_SCB, dPanelNormals_SCB, ...
%     dSamplePoints_SCB, dFaceVertices_SCB, dRayOffset, strRayData)
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
% strRayData           Optional prepared triangle data; retain five-input callers.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVisibleFraction    (N, 1)   Sun-visible sample fraction; back-facing faces are zero [-].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-09-2026  Pietro Califano, Codex gpt-6  Add standalone panel self-shadowing geometry.
% 30-09-2026  Pietro Califano, Codex gpt-6  Clarify ray tolerance and review geometry contracts.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% 08-10-2026  Pietro Califano, Codex (GPT-6)  Reuse prepared geometry for panel visibility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildTriangleRayData, ComputePreparedPanelVisibility.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dSunDir_SCB (3, 1) double {mustBeFinite}
    dPanelNormals_SCB (3, :) double {mustBeFinite}
    dSamplePoints_SCB (3, :, :) double {mustBeFinite}
    dFaceVertices_SCB (3, 3, :) double {mustBeFinite}
    dRayOffset (1, 1) double {mustBeFinite, mustBePositive}
    strRayData (1, 1) struct = struct()
end

arguments (Output)
    dVisibleFraction (:, 1) double
end

% Enforce matching face indices before entering the prepared numerical kernel.
ui32FaceCount = uint32(size(dPanelNormals_SCB, 2));

assert(size(dSamplePoints_SCB, 3) == ui32FaceCount && ...
    size(dFaceVertices_SCB, 3) == ui32FaceCount && size(dSamplePoints_SCB, 2) > 0, ...
    'ComputePanelSunVisibility:GeometrySizeMismatch', ...
    'Normals, samples and vertices must share face indices and nonempty quadrature.');

if ~isfield(strRayData, 'ui32TriangleCount')
    % Keep legacy inputs usable; prepared callers avoid rebuilding static edges.
    strPreparedRayData = BuildTriangleRayData(dFaceVertices_SCB, false);
else
    strPreparedRayData = strRayData;
end

assert(strPreparedRayData.ui32TriangleCount == ui32FaceCount, ...
    'ComputePanelSunVisibility:GeometrySizeMismatch', 'Prepared geometry must match face indices.');

strShadowData = struct('dSamplePoints_SCB', dSamplePoints_SCB, ...
    'dRayOffset', dRayOffset, 'strRayData', strPreparedRayData);
dVisibleFraction = ComputePreparedPanelVisibility(dSunDir_SCB, dPanelNormals_SCB, strShadowData);
end
