function [ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ComputePolyhedronFaceEdgeData(ui32FaceVertexIds, dVerticesPos) %#codegen
arguments
    ui32FaceVertexIds (:,3) uint32
    dVerticesPos      (:,3) double
end
%% PROTOTYPE
% [ui32EdgeVertexIds, dEdgeDyadics, dFaceDyadics] = ComputePolyhedronFaceEdgeData(ui32FaceVertexIds, dVerticesPos)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Precomputes the geometric data required by the Werner-Scheeres polyhedron
% gravity model: edge vertex IDs, edge dyadic tensors (Ee) and face dyadic
% tensors (Ff). This function is intended to be called ONCE at
% initialization, not per integration step. The outputs are cached and
% passed to EvalPolyhedronGrav at each evaluation.
%
% The input mesh must be:
% 1) triangulated,
% 2) closed and two-manifold, and
% 3) consistently OUTWARD-wound.
% The winding convention matters because EvalPolyhedronGrav uses the face
% ordering to compute oriented solid angles.
%
% REFERENCE:
% 1) Werner, R.A. and Scheeres, D.J. (1997), Exterior Gravitation of a
%    Polyhedron Derived and Compared with Harmonic and Mascon Gravitation
%    Representations of Asteroid 4769 Castalia. Celestial Mechanics and
%    Dynamical Astronomy, 65, 313-344.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32FaceVertexIds:  [nFaces x 3, uint32]  Triangle vertex indices (1-based)
% dVerticesPos:       [nVertices x 3, double] Vertex coordinates [LU]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% ui32EdgeVertexIds:  [nEdges x 2, uint32]       Unique edge vertex ID pairs
% dEdgeDyadics:       [3 x 3 x nEdges, double]   Edge dyadic tensor Ee per edge
% dFaceDyadics:       [3 x 3 x nFaces, double]   Face dyadic tensor Ff per face
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-03-2026    Pietro Califano    Codegen-ready implementation for SimulationGears,
%                                  ported and optimized from simulationUtils
%                                  (original: Fabio Ferrari, Carmine Buonagura).
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

ui32NumFaces = size(ui32FaceVertexIds, 1);

%% Validate global face orientation
dTriVerts1 = dVerticesPos(ui32FaceVertexIds(:,1), :);
dTriVerts2 = dVerticesPos(ui32FaceVertexIds(:,2), :);
dTriVerts3 = dVerticesPos(ui32FaceVertexIds(:,3), :);
dSignedVolume = sum(sum(dTriVerts1 .* cross(dTriVerts2, dTriVerts3, 2), 2)) / 6.0;

if ~(dSignedVolume > 0.0)
    error('ComputePolyhedronFaceEdgeData:InvalidFaceOrientation', ...
        'The face winding must define a closed polyhedron with outward normals.');
end

% Edge vectors for all faces (vectorized)
dEdge12 = dVerticesPos(ui32FaceVertexIds(:,2), :) - dVerticesPos(ui32FaceVertexIds(:,1), :); % [nFaces x 3]
dEdge13 = dVerticesPos(ui32FaceVertexIds(:,3), :) - dVerticesPos(ui32FaceVertexIds(:,1), :);
dEdge23 = dVerticesPos(ui32FaceVertexIds(:,3), :) - dVerticesPos(ui32FaceVertexIds(:,2), :);

% Normalize edge vectors
dEdge12Norm = dEdge12 ./ sqrt(sum(dEdge12.^2, 2));
dEdge13Norm = dEdge13 ./ sqrt(sum(dEdge13.^2, 2));
dEdge23Norm = dEdge23 ./ sqrt(sum(dEdge23.^2, 2));

% Face unit normals: cross(e12, e13), then normalize
dFaceNormals = cross(dEdge12Norm, dEdge13Norm, 2);
dFaceNormals = dFaceNormals ./ sqrt(sum(dFaceNormals.^2, 2)); % [nFaces x 3]

% Edge normals for each face edge (outward, in plane of face)
dEdgeNorm12 = cross(dEdge12Norm, dFaceNormals, 2);
dEdgeNorm12 = dEdgeNorm12 ./ sqrt(sum(dEdgeNorm12.^2, 2));

dEdgeNorm23 = cross(dEdge23Norm, dFaceNormals, 2);
dEdgeNorm23 = dEdgeNorm23 ./ sqrt(sum(dEdgeNorm23.^2, 2));

dEdgeNorm31 = cross(-dEdge13Norm, dFaceNormals, 2);
dEdgeNorm31 = dEdgeNorm31 ./ sqrt(sum(dEdgeNorm31.^2, 2));

%% Build unique-edge table and adjacent-face map without triangulation
ui32FaceIds = uint32((1:ui32NumFaces)');
ui32NumAllEdges = 3 * ui32NumFaces;

ui32AllEdgePairs = zeros(ui32NumAllEdges, 2, 'uint32');
ui32AllFaceIds = zeros(ui32NumAllEdges, 1, 'uint32');
ui8AllLocalEdgeIds = zeros(ui32NumAllEdges, 1, 'uint8');
ui32AllEdgeStarts = zeros(ui32NumAllEdges, 1, 'uint32');
ui32AllEdgeEnds = zeros(ui32NumAllEdges, 1, 'uint32');

idxRows12 = (1:ui32NumFaces)';
idxRows23 = ui32NumFaces + (1:ui32NumFaces)';
idxRows31 = 2 * ui32NumFaces + (1:ui32NumFaces)';

ui32Vert1 = ui32FaceVertexIds(:, 1);
ui32Vert2 = ui32FaceVertexIds(:, 2);
ui32Vert3 = ui32FaceVertexIds(:, 3);

ui32AllEdgePairs(idxRows12, :) = [min(ui32Vert1, ui32Vert2), max(ui32Vert1, ui32Vert2)];
ui32AllEdgePairs(idxRows23, :) = [min(ui32Vert2, ui32Vert3), max(ui32Vert2, ui32Vert3)];
ui32AllEdgePairs(idxRows31, :) = [min(ui32Vert3, ui32Vert1), max(ui32Vert3, ui32Vert1)];

ui32AllFaceIds(idxRows12) = ui32FaceIds;
ui32AllFaceIds(idxRows23) = ui32FaceIds;
ui32AllFaceIds(idxRows31) = ui32FaceIds;

ui8AllLocalEdgeIds(idxRows12) = uint8(1);
ui8AllLocalEdgeIds(idxRows23) = uint8(2);
ui8AllLocalEdgeIds(idxRows31) = uint8(3);

ui32AllEdgeStarts(idxRows12) = ui32Vert1;
ui32AllEdgeEnds(idxRows12) = ui32Vert2;
ui32AllEdgeStarts(idxRows23) = ui32Vert2;
ui32AllEdgeEnds(idxRows23) = ui32Vert3;
ui32AllEdgeStarts(idxRows31) = ui32Vert3;
ui32AllEdgeEnds(idxRows31) = ui32Vert1;

ui64EdgeKeyBase = uint64(size(dVerticesPos, 1)) + uint64(1);
ui64EdgeKeys = uint64(ui32AllEdgePairs(:,1)) * ui64EdgeKeyBase + uint64(ui32AllEdgePairs(:,2));
[ui64EdgeKeys, idxSort] = sort(ui64EdgeKeys);

ui32AllEdgePairs = ui32AllEdgePairs(idxSort, :);
ui32AllFaceIds = ui32AllFaceIds(idxSort);
ui8AllLocalEdgeIds = ui8AllLocalEdgeIds(idxSort);
ui32AllEdgeStarts = ui32AllEdgeStarts(idxSort);
ui32AllEdgeEnds = ui32AllEdgeEnds(idxSort);

ui32EdgeVertexIds = zeros(ui32NumAllEdges, 2, 'uint32');
dEdgeDyadics = zeros(3, 3, ui32NumAllEdges);
ui32NumEdges = uint32(0);

% Iterate through sorted edge list to find unique edges and their adjacent faces
idxAll = 1;
while idxAll <= ui32NumAllEdges

    if (idxAll == ui32NumAllEdges) || ...
            (ui64EdgeKeys(idxAll) ~= ui64EdgeKeys(idxAll + 1)) || ...
            ((idxAll + 2) <= ui32NumAllEdges && ui64EdgeKeys(idxAll + 1) == ui64EdgeKeys(idxAll + 2))
        error('ComputePolyhedronFaceEdgeData:NonManifoldMesh', ...
            'Each edge must be shared by exactly two faces in a closed triangular mesh.');
    end

    ui32FaceA = ui32AllFaceIds(idxAll);
    ui32FaceB = ui32AllFaceIds(idxAll + 1);
    
    if ui32FaceA == ui32FaceB
        error('ComputePolyhedronFaceEdgeData:DegenerateMesh', ...
            'A valid polyhedron edge cannot belong to the same face twice.');
    end

    if ~(ui32AllEdgeStarts(idxAll) == ui32AllEdgeEnds(idxAll + 1) && ...
            ui32AllEdgeEnds(idxAll) == ui32AllEdgeStarts(idxAll + 1))
        error('ComputePolyhedronFaceEdgeData:InconsistentFaceWinding', ...
            'Adjacent faces must traverse each shared edge in opposite directions.');
    end

    dNormalA = dFaceNormals(ui32FaceA, :)';
    dNormalB = dFaceNormals(ui32FaceB, :)';

    dEdgeNormA = GetEdgeNormalFromLocalId(ui32FaceA, ui8AllLocalEdgeIds(idxAll), ...
        dEdgeNorm12, dEdgeNorm23, dEdgeNorm31);
    dEdgeNormB = GetEdgeNormalFromLocalId(ui32FaceB, ui8AllLocalEdgeIds(idxAll + 1), ...
        dEdgeNorm12, dEdgeNorm23, dEdgeNorm31);

    ui32NumEdges = ui32NumEdges + 1;
    ui32EdgeVertexIds(ui32NumEdges, :) = ui32AllEdgePairs(idxAll, :);
    dEdgeDyadics(:,:,ui32NumEdges) = dNormalA * dEdgeNormA' + dNormalB * dEdgeNormB';

    idxAll = idxAll + 2;
end

ui32EdgeVertexIds = ui32EdgeVertexIds(1:ui32NumEdges, :);
dEdgeDyadics = dEdgeDyadics(:,:,1:ui32NumEdges);

%% Compute face dyadic tensors Ff = nF * nF'
dFaceDyadics = zeros(3, 3, ui32NumFaces);
for idxFace = 1:ui32NumFaces
    dNormalF = dFaceNormals(idxFace, :)';
    dFaceDyadics(:,:,idxFace) = dNormalF * dNormalF';
end

end

%% LOCAL FUNCTIONS
function dEdgeNormalOut = GetEdgeNormalFromLocalId(idxFace, ui8LocalEdgeId, dEdgeNorm12, dEdgeNorm23, dEdgeNorm31)
if ui8LocalEdgeId == uint8(1)
    dEdgeNormalOut = dEdgeNorm12(idxFace, :)';
elseif ui8LocalEdgeId == uint8(2)
    dEdgeNormalOut = dEdgeNorm23(idxFace, :)';
else
    dEdgeNormalOut = dEdgeNorm31(idxFace, :)';
end
end
