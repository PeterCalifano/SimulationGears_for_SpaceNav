function [dVerticesPos, ui32FaceVertexIds] = ...
        CompactShapeMeshVertices(dVerticesPos, ui32FaceVertexIds)
%% SIGNATURE
% [dVerticesPos, ui32FaceVertexIds] = CompactShapeMeshVertices(dVerticesPos, ui32FaceVertexIds)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Remove unreferenced vertices from row-major geometry without welding, reordering faces or
% changing winding. Retained vertices stay in source order. Use this after object selection so
% excluded geometry cannot influence bounds, normalization radii or mesh-scaled repair thresholds.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dVerticesPos        N-by-3 source positions in caller-declared length units.
% ui32FaceVertexIds   F-by-3 one-based source indices; zero/out-of-range indices are rejected.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVerticesPos        Referenced positions, preserving coordinates and units exactly.
% ui32FaceVertexIds   Same ordered faces with compact one-based vertex indices.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-09-2026  Pietro Califano     Share deterministic vertex compaction after OBJ selection.
% 29-09-2026  Pietro Califano, Codex gpt-6    Document source-order and index invariants.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Base MATLAB array operations.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    dVerticesPos (:,3) double
    ui32FaceVertexIds (:,3) uint32
end
arguments (Output)
    dVerticesPos (:,3) double
    ui32FaceVertexIds (:,3) uint32
end

% Reject invalid references before using face indices to address source vertices.
if any(ui32FaceVertexIds == 0, 'all') || ...
        any(ui32FaceVertexIds > size(dVerticesPos, 1), 'all')
    error('CompactShapeMeshVertices:BadFaceIndex', ...
        'Face indices must address an existing source vertex.');
end

% Keep referenced vertices in source order and remap faces without changing their winding.
ui32UsedVertices = unique(ui32FaceVertexIds(:));
ui32VertexMap = zeros(size(dVerticesPos, 1), 1, 'uint32');
ui32VertexMap(double(ui32UsedVertices)) = uint32(1):uint32(numel(ui32UsedVertices));
ui32FaceVertexIds = reshape(ui32VertexMap(double(ui32FaceVertexIds)), size(ui32FaceVertexIds));
dVerticesPos = dVerticesPos(double(ui32UsedVertices), :);
end
