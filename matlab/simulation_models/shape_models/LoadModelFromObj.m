function [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
    ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = LoadModelFromObj(charObjFilePath, bVertFacesOnly) %#codegen
arguments
    charObjFilePath (1,1) string {mustBeA(charObjFilePath, ["string", "char"])}
    bVertFacesOnly (1,1) {islogical} = true;
end
%% SIGNATURE
% [ui32TrianglesIndex, dVerticesCoords, dTexCoords, ...
%  ui32TrianglesTexIndex, dNormals, ui32TrianglesNormalsIndex] = LoadModelFromObj(charObjFilePath, bVertFacesOnly) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% [ui32TrianglesIndex, dVerticesCoords] = LoadModelFromObj(charObjFilePath) reads the vertices and the
% triangles data as specified in the input Wavefront .obj file.
% This implementation uses vectorized regexp and sscanf on the entire file content, avoiding
% per-line loops and dynamic allocation. Output formats:
%     ui32TrianglesIndex    - R-by-3 uint32 array of face indices (v/vt/vn)
%     dVerticesCoords       - M-by-3 double array of vertex coordinates
%     dTexCoords            - P-by-2 double array of texture coordinates (if present)
%     dNormals              - Q-by-3 double array of normals (if present)
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-01-2025    Pietro Califano     Function implemented for general obj format loading
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

tic
% Check extension
[~,~, charFileExt] = fileparts(charObjFilePath);

if ~strcmpi(charFileExt, '.obj')
    error('LoadModelFromObj:InvalidExtension', 'Input file must have .obj extension.');
end

if not(isfile(charObjFilePath))
    error('LoadModelFromObj:FileNotFound', 'Cannot find file: %s', charObjFilePath);
end

% Read entire file as text
charFileText = fileread(charObjFilePath);

% Vertex lines: 'v x y z'
vPattern = '^v\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)';
vLines = regexp(charFileText, vPattern, 'tokens', 'lineanchors');

if ~isempty(vLines)
    vTokens = vertcat(vLines{:});
    dVerticesCoords = str2double(vTokens);
    dVerticesCoords = reshape(dVerticesCoords, 3, [])';
else
    dVerticesCoords = zeros(0,3);
end

% Texture-coordinate lines: 'vt u v'
vtPattern = '^vt\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)';
vtLines = regexp(charFileText, vtPattern, 'tokens', 'lineanchors');

if ~isempty(vtLines) && not(bVertFacesOnly)
    vtTokens = vertcat(vtLines{:});
    dTexCoords = str2double(vtTokens);
    dTexCoords = reshape(dTexCoords, 2, [])';
else
    dTexCoords = zeros(0,2);
end

% Normal lines: 'vn nx ny nz'
vnPattern = '^vn\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)\s+([\-\d\.eE\+]+)';
vnLines = regexp(charFileText, vnPattern, 'tokens', 'lineanchors');


if ~isempty(vnLines) && not(bVertFacesOnly)
    vnTokens = vertcat(vnLines{:});
    dNormals = str2double(vnTokens);
    dNormals = reshape(dNormals, 3, [])';
else
    dNormals = zeros(0,3);
end


% Defaults
ui32TrianglesIndex          = zeros(0,3,'uint32');
ui32TrianglesTexIndex       = zeros(0,3,'uint32');
ui32TrianglesNormalsIndex   = zeros(0,3,'uint32');

% Determine face format by presence of vt/vn
bHasVT = ~isempty(dTexCoords);
bHasVN = ~isempty(dNormals);

% Build regex and index maps
if bHasVT && bHasVN
    charPattern = '^f\s+(\d+)/(\d+)/(\d+)\s+(\d+)/(\d+)/(\d+)\s+(\d+)/(\d+)/(\d+)';
    vidx = 1:3:9; tidx = 2:3:9; nidx = 3:3:9;

elseif bHasVT
    charPattern = '^f\s+(\d+)/(\d+)\s+(\d+)/(\d+)\s+(\d+)/(\d+)';
    vidx = 1:2:6; tidx = 2:2:6; nidx = [];

elseif bHasVN
    charPattern = '^f\s+(\d+)//(\d+)\s+(\d+)//(\d+)\s+(\d+)//(\d+)';
    vidx = 1:2:6; tidx = [];    nidx = 2:2:6;

else
    charPattern = '^f\s+(\d+)\s+(\d+)\s+(\d+)';
    vidx = 1:3; tidx = [];    nidx = [];
end

% Parse face lines
fTokens = regexp(charFileText, charPattern, 'tokens', 'lineanchors');

if ~isempty(fTokens) && not(bVertFacesOnly)

    dValues = str2double(vertcat(fTokens{:}));
    dFacesMatrix = reshape(dValues, numel(vidx) + numel(tidx) + numel(nidx), [])';
    ui32TrianglesIndex = uint32(dFacesMatrix(:, vidx));

    if bHasVT, ui32TrianglesTexIndex = uint32(dFacesMatrix(:, tidx)); end
    if bHasVN, ui32TrianglesNormalsIndex   = uint32(dFacesMatrix(:, nidx)); end
end

dElapsedTime = toc;
fprintf("\nFile obj loaded in %.5g seconds\n", dElapsedTime);
end
