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

%% Compute face and edge unit normals
ui32NumFaces = uint32(size(ui32FaceVertexIds, 1));

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

%% Extract unique edges via triangulation
objTriang = triangulation(double(ui32FaceVertexIds), dVerticesPos);
ui32EdgeVertexIds = uint32(edges(objTriang));
ui32NumEdges = uint32(size(ui32EdgeVertexIds, 1));

%% Compute edge dyadic tensors Ee = nA * nijA' + nB * njiB'
dEdgeDyadics = zeros(3, 3, ui32NumEdges);

% Process each edge
for idxEdge = 1:ui32NumEdges

    ui32VertI = ui32EdgeVertexIds(idxEdge, 1);
    ui32VertJ = ui32EdgeVertexIds(idxEdge, 2);

    % Find the two faces sharing this edge
    bFacesWithI = (ui32FaceVertexIds == ui32VertI);
    bFacesWithJ = (ui32FaceVertexIds == ui32VertJ);
    % A face contains this edge if it has both vertices
    ui32FaceRowHasI = find(any(bFacesWithI, 2));
    ui32FaceRowHasJ = find(any(bFacesWithJ, 2));

    % Identify face A (first shared) and face B (second shared)
    ui32SharedFaces = intersect(ui32FaceRowHasI, ui32FaceRowHasJ);

    ui32FaceA = ui32SharedFaces(1);
    ui32FaceB = ui32SharedFaces(2);

    % Face normals for A and B
    dNormalA = dFaceNormals(ui32FaceA, :)';
    dNormalB = dFaceNormals(ui32FaceB, :)';

    % Identify which local edge in face A contains (vertI, vertJ)
    dEdgeNormIJinA = GetEdgeNormalInFace(ui32FaceVertexIds(ui32FaceA,:), ...
        ui32VertI, ui32VertJ, ...
        dEdgeNorm12(ui32FaceA,:)', dEdgeNorm23(ui32FaceA,:)', dEdgeNorm31(ui32FaceA,:)');

    % Same for face B
    dEdgeNormIJinB = GetEdgeNormalInFace(ui32FaceVertexIds(ui32FaceB,:), ...
        ui32VertI, ui32VertJ, ...
        dEdgeNorm12(ui32FaceB,:)', dEdgeNorm23(ui32FaceB,:)', dEdgeNorm31(ui32FaceB,:)');

    % Edge dyadic: Ee = nA * nijA' + nB * njiB'
    dEdgeDyadics(:,:,idxEdge) = dNormalA * dEdgeNormIJinA' + dNormalB * dEdgeNormIJinB';
end

%% Compute face dyadic tensors Ff = nF * nF'
dFaceDyadics = zeros(3, 3, ui32NumFaces);
for idxFace = 1:ui32NumFaces
    dNormalF = dFaceNormals(idxFace, :)';
    dFaceDyadics(:,:,idxFace) = dNormalF * dNormalF';
end

end

%% LOCAL FUNCTIONS
function dEdgeNormalOut = GetEdgeNormalInFace(ui32FaceVerts, ui32VertI, ui32VertJ, dEN12, dEN23, dEN31)
%GetEdgeNormalInFace Retrieve the edge normal for edge (vertI, vertJ) in a given face.
% Face vertices are [v1, v2, v3]. Edge 12 connects v1-v2, edge 23 connects v2-v3,
% edge 31 connects v3-v1.
idxI = find(ui32FaceVerts == ui32VertI, 1);
idxJ = find(ui32FaceVerts == ui32VertJ, 1);

ui32PairId = sort([idxI, idxJ]);

if isequal(ui32PairId, [1, 2])
    dEdgeNormalOut = dEN12;
elseif isequal(ui32PairId, [2, 3])
    dEdgeNormalOut = dEN23;
else
    dEdgeNormalOut = dEN31;
end
end
