function [dVolume, dCoM] = ComputeMeshModelVolumeAndCoM(ui32FaceVertexIds, dVerticesPos)
%% PROTOTYPE
% [dVolume, dCoM] = ComputeMeshModelVolumeAndCoM(ui32FaceVertexIds, dVerticesPos)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Computes the signed enclosed volume and center of mass of a closed,
% outward-wound triangular mesh with uniform density.
%
% The mesh is decomposed into signed tetrahedra formed by each face and the
% origin. The returned center of mass is therefore expressed in the same
% frame and length unit as dVerticesPos.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% ui32FaceVertexIds:    [nFaces x 3]      Triangle vertex indices.
% dVerticesPos:         [nVertices x 3]   Vertex coordinates.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dVolume:              [1]               Signed enclosed mesh volume.
% dCoM:                 [3 x 1]           Uniform-density center of mass.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 26-04-2026    Pietro Califano     Extract mesh volume/CoM utility from SH polyhedron fitter.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

if ~isa(ui32FaceVertexIds, 'uint32') || size(ui32FaceVertexIds, 2) ~= 3
    error('ComputeMeshModelVolumeAndCoM:InvalidFaces', ...
        'ui32FaceVertexIds must be an nFaces-by-3 uint32 array.');
end

if ~isa(dVerticesPos, 'double') || size(dVerticesPos, 2) ~= 3
    error('ComputeMeshModelVolumeAndCoM:InvalidVertices', ...
        'dVerticesPos must be an nVertices-by-3 double array.');
end

dVerts1 = dVerticesPos(ui32FaceVertexIds(:, 1), :);
dVerts2 = dVerticesPos(ui32FaceVertexIds(:, 2), :);
dVerts3 = dVerticesPos(ui32FaceVertexIds(:, 3), :);

dSignedTetraVolumes = sum(dVerts1 .* cross(dVerts2, dVerts3, 2), 2) / 6.0;
dVolume = sum(dSignedTetraVolumes);

if ~(dVolume > 0.0)
    error('ComputeMeshModelVolumeAndCoM:InvalidVolume', ...
        'The mesh must define a positive enclosed volume.');
end

dCentroidNumerator = sum(((dVerts1 + dVerts2 + dVerts3) / 4.0) .* dSignedTetraVolumes, 1);
dCoM = (dCentroidNumerator / dVolume).';

end
