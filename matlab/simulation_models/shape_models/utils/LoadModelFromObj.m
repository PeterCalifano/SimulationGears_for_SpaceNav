function [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
        ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = LoadModelFromObj( ...
        charObjFilePath, bVertFacesOnly)
%% SIGNATURE
% [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
%  ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = ...
%     LoadModelFromObj(charObjFilePath, bVertFacesOnly)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Deprecated compatibility entry point for OBJ loading. The authoritative
% implementation is CShapeModel.LoadModelFromObj.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charObjFilePath:       Path to a Wavefront OBJ file.
% bVertFacesOnly:        Load only geometry when true.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% ui32TrianglesIndex:        Triangle vertex indices as 3-by-F uint32.
% dVerticesCoords:           Vertex coordinates as 3-by-N double.
% dTexCoords:                Texture coordinates when requested.
% ui32TrianglesTexIndex:     Triangle texture-coordinate indices.
% dNormals:                  Vertex normals when requested.
% ui32TrianglesNormalsIndex: Triangle normal indices.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-01-2025  Pietro Califano     First implementation for general OBJ loading.
% 28-08-2026  Pietro Califano     Replace duplicate parser with class forwarder.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% CShapeModel.LoadModelFromObj
% -------------------------------------------------------------------------------------------------------------

arguments(Input)
    charObjFilePath (1,1) string {mustBeA(charObjFilePath, ["string", "char"])}
    bVertFacesOnly (1,1) logical = true
end

arguments(Output)
    ui32TrianglesIndex uint32
    dVerticesCoords double
    dTexCoords double
    ui32TrianglesTexIndex uint32
    dNormals double
    ui32TrianglesNormalsIndex uint32
end

% Preserve the deprecated API while keeping the class method as the sole parser implementation.
[ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
    ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = ...
    CShapeModel.LoadModelFromObj(charObjFilePath, bVertFacesOnly);
end
