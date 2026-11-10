function [dFaceVertices_SCB, dSamplePoints_SCB] = BuildPanelSrpShadowData(strPanel, ui32ShadowLevel)
%% SIGNATURE
% [dFaceVertices_SCB, dSamplePoints_SCB] = BuildPanelSrpShadowData(strPanel, ui32ShadowLevel)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Prepare equal-area triangle-centroid quadrature for spacecraft self-shadowing.
% Preserve the supplied face ordering and optical normals. Subdivide each face
% into 4^level equal-area triangles in the input geometry's declared length
% unit. Preserve that unit in both outputs; use the same unit for ray offsets.
% LUT generation supplies metre geometry; dynamics preparation may use kilometres.
% Example: [dVertices, dSamples] = BuildPanelSrpShadowData(strPanel, uint32(3));
% Output: Three vertices and 64 equal-area sample points per original triangle.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strPanel            Prepared vertices and triangle indices in one length unit.
% ui32ShadowLevel     Subdivision level from zero through five.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dFaceVertices_SCB   (3, 3, N) original triangle vertices [panel length unit].
% dSamplePoints_SCB   (3, 4^level, N) equal-area samples [panel length unit].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-09-2026  Pietro Califano, Codex gpt-6  Move shared geometry preparation to its owner.
% 01-10-2026  Pietro Califano, Codex gpt-6  Clarify variable roles and separate computation steps.
% 04-10-2026  Pietro Califano, Codex GPT-6  Clarify unit-preserving truth preparation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None; host-side preparation for ComputePanelSunVisibility.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strPanel (1, 1) struct
    ui32ShadowLevel (1, 1) uint32
end

arguments (Output)
    dFaceVertices_SCB (3, 3, :) double
    dSamplePoints_SCB (3, :, :) double
end

assert(ui32ShadowLevel <= 5, 'BuildPanelSrpShadowData:ExcessiveSubdivision', ...
    'Use a shadow subdivision depth of at most five.');

% Refine one barycentric template into equal-area triangles for every mesh face.
dBarycentricTriangles = eye(3);
for ui32Depth = uint32(1):ui32ShadowLevel

    dChildBarycentricTriangles = zeros(3, 3, 4 * size(dBarycentricTriangles, 3));

    for ui32Triangle = uint32(1):uint32(size(dBarycentricTriangles, 3))
        dParentBarycentricTriangle = dBarycentricTriangles(:, :, ui32Triangle);
        dEdgeBarycentricMidpoints = (dParentBarycentricTriangle + ...
            dParentBarycentricTriangle(:, [2, 3, 1])) / 2;

        % Split the parent at its edge midpoints into four equal-area children.
        dChildBarycentricTriangles(:, :, 4 * ui32Triangle - 3) = ...
            [dParentBarycentricTriangle(:, 1), ...
             dEdgeBarycentricMidpoints(:, 1), dEdgeBarycentricMidpoints(:, 3)];
        dChildBarycentricTriangles(:, :, 4 * ui32Triangle - 2) = ...
            [dEdgeBarycentricMidpoints(:, 1), ...
             dParentBarycentricTriangle(:, 2), dEdgeBarycentricMidpoints(:, 2)];
        dChildBarycentricTriangles(:, :, 4 * ui32Triangle - 1) = ...
            [dEdgeBarycentricMidpoints(:, 3), ...
             dEdgeBarycentricMidpoints(:, 2), dParentBarycentricTriangle(:, 3)];
        dChildBarycentricTriangles(:, :, 4 * ui32Triangle) = dEdgeBarycentricMidpoints;
    end

    dBarycentricTriangles = dChildBarycentricTriangles;
end

% Map the shared quadrature into each original face without changing face indices.
dSampleBarycentricCoords = reshape(mean(dBarycentricTriangles, 2), 3, []);
ui32FaceCount = uint32(size(strPanel.ui32FaceVertexIds, 1));
dFaceVertices_SCB = zeros(3, 3, ui32FaceCount);
dSamplePoints_SCB = zeros(3, size(dSampleBarycentricCoords, 2), ui32FaceCount);

for ui32FaceIndex = uint32(1):ui32FaceCount
    dFaceVertices_SCB(:, :, ui32FaceIndex) = ...
        strPanel.dVerticesPos(strPanel.ui32FaceVertexIds(ui32FaceIndex, :), :).';
    dSamplePoints_SCB(:, :, ui32FaceIndex) = ...
        dFaceVertices_SCB(:, :, ui32FaceIndex) * dSampleBarycentricCoords;
end
end
