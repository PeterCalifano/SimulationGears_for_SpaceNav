function strShapeMesh = LoadShapeMesh(charMeshPath, options)
%% SIGNATURE
% strShapeMesh = LoadShapeMesh(charMeshPath, options)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Load validated geometry from an OBJ, ASCII STL, or binary STL file without
% changing its frame or length unit. Optional repair welds exactly duplicated
% vertices, removes degenerate triangles, and compacts unused vertices.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charMeshPath:          Path to the source mesh file.
% options.bRepairMesh:   Apply deterministic geometry repair when true.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strShapeMesh:          Struct with row-major dVerticesPos, row-major
%                        ui32FaceVertexIds, source path, and geometry counts.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 28-08-2026  Pietro Califano     Promote validated shared OBJ/STL mesh loading.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Base MATLAB file I/O and geometry functions.
% -------------------------------------------------------------------------------------------------------------

arguments(Input)
    charMeshPath (1, :) char
    options.bRepairMesh (1, 1) logical = true
end

arguments(Output)
    strShapeMesh (1, 1) struct
end

% Reject missing input before format dispatch so every reader shares one boundary error.
if ~isfile(charMeshPath)
    error('LoadShapeMesh:MissingFile', 'Shape mesh file does not exist: %s', charMeshPath);
end

% Dispatch by the declared format; STL encoding is resolved from the file content.
[~, ~, charExtension] = fileparts(charMeshPath);
switch lower(charExtension)
    case '.obj'
        [dVerticesPos, ui32FaceVertexIds] = ReadObj_(charMeshPath);
    case '.stl'
        [dVerticesPos, ui32FaceVertexIds] = ReadStl_(charMeshPath);
    otherwise
        error('LoadShapeMesh:UnsupportedExtension', ...
            'Unsupported shape mesh extension: %s', charExtension);
end

% Validate raw parser output before the optional index-changing repair step.
ValidateMesh_(dVerticesPos, ui32FaceVertexIds, charMeshPath);

if options.bRepairMesh
    [dVerticesPos, ui32FaceVertexIds] = RepairMesh_( ...
        dVerticesPos, ui32FaceVertexIds, charMeshPath);
end

% Expose one row-major geometry contract independently of the source format.
strShapeMesh = struct();
strShapeMesh.ui32FaceVertexIds = ui32FaceVertexIds;
strShapeMesh.dVerticesPos = dVerticesPos;
strShapeMesh.charSourcePath = charMeshPath;
strShapeMesh.ui32NumVertices = uint32(size(dVerticesPos, 1));
strShapeMesh.ui32NumFaces = uint32(size(ui32FaceVertexIds, 1));
end

function [dVerticesPos, ui32FaceVertexIds] = ReadObj_(charMeshPath)
%% DESCRIPTION
% Parse OBJ geometry records and triangulate simple planar polygon faces.
% -------------------------------------------------------------------------------------------------------------

% Preserve the vectorized path for the common positive-index triangular format.
[bUsedFastPath, dVerticesPos, ui32FaceVertexIds] = TryReadTriangularObjFast_(charMeshPath);
if bUsedFastPath
    return
end

% Fall back to record-wise parsing for polygons, slash indices, and relative indices.
i32FileId = fopen(charMeshPath, 'r');
if i32FileId < 0
    error('LoadShapeMesh:OpenFailed', 'Could not open OBJ file: %s', charMeshPath);
end
objCleanup = onCleanup(@() fclose(i32FileId));

% Grow temporary storage geometrically to avoid per-record reallocation.
dVerticesTmp = zeros(1000, 3);
ui32FacesTmp = zeros(1000, 3, 'uint32');
ui32NumVertices = uint32(0);
ui32NumFaces = uint32(0);

while true
    charLine = fgetl(i32FileId);
    if ~ischar(charLine)
        break
    end

    charLine = strtrim(charLine);
    if isempty(charLine) || startsWith(charLine, '#')
        continue
    end

    % Vertices must be materialized in source order for later relative face indices.
    if IsObjRecord_(charLine, 'v')
        dVertex = sscanf(charLine(2:end), '%f');
        if numel(dVertex) < 3 || any(~isfinite(dVertex(1:3)))
            error('LoadShapeMesh:BadObjVertex', ...
                'OBJ vertex records must contain three finite coordinates.');
        end

        ui32NumVertices = ui32NumVertices + uint32(1);
        dVerticesTmp = EnsureDoubleRows_(dVerticesTmp, ui32NumVertices);
        dVerticesTmp(double(ui32NumVertices), :) = transpose(dVertex(1:3));
        continue
    end

    % Keep triangles unchanged and ear-clip only faces that contain more than three vertices.
    if IsObjRecord_(charLine, 'f')
        ui32PolygonVertexIds = ParseObjFace_(charLine(2:end), ui32NumVertices);
        if numel(ui32PolygonVertexIds) < 3
            error('LoadShapeMesh:BadObjFace', ...
                'OBJ faces must contain at least three vertex indices.');
        end

        if numel(ui32PolygonVertexIds) == 3
            ui32NewFaces = reshape(ui32PolygonVertexIds, 1, 3);
        else
            dVerticesDefined = dVerticesTmp(1:double(ui32NumVertices), :);
            ui32NewFaces = TriangulateObjPolygon_( ...
                ui32PolygonVertexIds, dVerticesDefined);
        end

        ui32NumNewFaces = uint32(size(ui32NewFaces, 1));
        ui32FacesTmp = EnsureUintRows_(ui32FacesTmp, ui32NumFaces + ui32NumNewFaces);
        ui32DestinationRows = ui32NumFaces + uint32(1):ui32NumFaces + ui32NumNewFaces;
        ui32FacesTmp(double(ui32DestinationRows), :) = ui32NewFaces;
        ui32NumFaces = ui32NumFaces + ui32NumNewFaces;
        continue
    end

    % Accept standard non-geometry payload without introducing texture or material ownership.
    if IsIgnoredObjRecord_(charLine)
        continue
    end

    charRecordType = regexp(charLine, '^\S+', 'match', 'once');
    error('LoadShapeMesh:UnsupportedObjRecord', ...
        'Unsupported OBJ record "%s".', charRecordType);
end

% Trim unused geometric-growth capacity before returning the parsed geometry.
dVerticesPos = dVerticesTmp(1:double(ui32NumVertices), :);
ui32FaceVertexIds = ui32FacesTmp(1:double(ui32NumFaces), :);
end

function [bUsedFastPath, dVerticesPos, ui32FaceVertexIds] = ...
        TryReadTriangularObjFast_(charMeshPath)
%% DESCRIPTION
% Parse ordinary positive-index triangular OBJ geometry using vectorized
% whole-file operations. Return false for inputs requiring the general path.
% -------------------------------------------------------------------------------------------------------------

% Leave the fallback result valid until the complete fast-path contract is proven.
bUsedFastPath = false;
dVerticesPos = zeros(0, 3);
ui32FaceVertexIds = zeros(0, 3, 'uint32');

charFileText = fileread(charMeshPath);

% Extract only the geometry records needed by the shared reader.
cellVertexLines = regexp(charFileText, '^v[ \t]+[^\r\n]*$', 'match', 'lineanchors');
cellFaceLines = regexp(charFileText, '^f[ \t]+[^\r\n]*$', 'match', 'lineanchors');
if isempty(cellVertexLines) || isempty(cellFaceLines)
    return
end

% Defer slash indices, relative indices, and polygons to the general parser.
charNonTriangularFace = regexp(charFileText, ...
    '^f[ \t]+(?!\d+[ \t]+\d+[ \t]+\d+[ \t]*$)[^\r\n]*$', ...
    'match', 'once', 'lineanchors');
if ~isempty(charNonTriangularFace)
    return
end

% Parse each record family in one bulk operation and reject partial conversions.
charVertexBlock = sprintf('%s\n', cellVertexLines{:});
dVerticesColumns = sscanf(charVertexBlock, 'v %f %f %f\n', [3, Inf]);
if size(dVerticesColumns, 2) ~= numel(cellVertexLines) || ...
        any(~isfinite(dVerticesColumns), 'all')
    return
end

charFaceBlock = sprintf('%s\n', cellFaceLines{:});
dFaceColumns = sscanf(charFaceBlock, 'f %u %u %u\n', [3, Inf]);
if size(dFaceColumns, 2) ~= numel(cellFaceLines)
    return
end

% Validate index bounds before narrowing the parsed doubles to uint32.
if any(dFaceColumns < 1.0, 'all') || max(dFaceColumns, [], 'all') > size(dVerticesColumns, 2)
    error('LoadShapeMesh:BadFaceIndex', ...
        'OBJ face index exceeds the vertices defined in the file.');
end

dVerticesPos = transpose(dVerticesColumns);
ui32FaceVertexIds = uint32(transpose(dFaceColumns));
bUsedFastPath = true;
end

function bMatches = IsObjRecord_(charLine, charRecordName)
bMatches = startsWith(charLine, [charRecordName, ' ']) || ...
    startsWith(charLine, [charRecordName, sprintf('\t')]);
end

function bIgnored = IsIgnoredObjRecord_(charLine)
cellIgnoredRecords = {'vt', 'vn', 'vp', 'o', 'g', 's', 'usemtl', 'mtllib'};
bIgnored = false;
for ui32RecordIdx = uint32(1):uint32(numel(cellIgnoredRecords))
    if IsObjRecord_(charLine, cellIgnoredRecords{double(ui32RecordIdx)})
        bIgnored = true;
        return
    end
end
end

function ui32VertexIds = ParseObjFace_(charFaceText, ui32NumVertices)
cellFaceTokens = regexp(strtrim(charFaceText), '\s+', 'split');
ui32VertexIds = zeros(numel(cellFaceTokens), 1, 'uint32');

for ui32TokenIdx = uint32(1):uint32(numel(cellFaceTokens))
    % Geometry uses the first slash-delimited field; texture and normal indices are ignored.
    cellTokenParts = regexp(cellFaceTokens{double(ui32TokenIdx)}, '/', 'split');
    dRawIndex = str2double(cellTokenParts{1});
    if ~isfinite(dRawIndex) || dRawIndex == 0.0 || dRawIndex ~= fix(dRawIndex)
        error('LoadShapeMesh:BadFaceIndex', ...
            'OBJ face indices must be finite nonzero integers.');
    end

    if dRawIndex < 0.0
        dRawIndex = double(ui32NumVertices) + 1.0 + dRawIndex;
    end

    % Relative indices are resolved against vertices defined before this face.
    if dRawIndex < 1.0 || dRawIndex > double(ui32NumVertices)
        error('LoadShapeMesh:BadFaceIndex', ...
            'OBJ face index exceeds the vertices defined before the face.');
    end

    ui32VertexIds(double(ui32TokenIdx)) = uint32(dRawIndex);
end
end

function ui32Triangles = TriangulateObjPolygon_(ui32PolygonVertexIds, dVerticesPos)
%% DESCRIPTION
% Ear-clip a simple planar OBJ polygon in its dominant projection plane.
% -------------------------------------------------------------------------------------------------------------

% Scale geometric tolerances to the polygon while retaining an absolute floor.
dPolygonVertices = dVerticesPos(double(ui32PolygonVertexIds), :);
dMeshScale = max(max(dPolygonVertices, [], 1) - min(dPolygonVertices, [], 1));
dLengthTolerance = max(100.0 .* eps(max(max(abs(dPolygonVertices), [], 'all'), 1.0)), ...
    1.0e-12 .* max(dMeshScale, 1.0));

% Newell's method supplies a stable polygon normal without assuming a convex face.
dNextVertices = dPolygonVertices([2:end, 1], :);
dNewellNormal = sum(cross(dPolygonVertices, dNextVertices, 2), 1);
dNormalNorm = norm(dNewellNormal);
if dNormalNorm <= dLengthTolerance * max(dMeshScale, 1.0)
    error('LoadShapeMesh:BadObjFace', ...
        'OBJ polygon is degenerate or self-intersecting.');
end

% Enforce planarity before reducing the polygon to a two-dimensional problem.
dUnitNormal = dNewellNormal ./ dNormalNorm;
dPlaneDistance = abs((dPolygonVertices - dPolygonVertices(1, :)) * transpose(dUnitNormal));
if max(dPlaneDistance) > dLengthTolerance
    error('LoadShapeMesh:BadObjFace', 'OBJ polygon vertices must be coplanar.');
end

% Drop the dominant normal dimension to maximize projected area and numerical stability.
[~, dDropDimension] = max(abs(dUnitNormal));
ui32KeepDimensions = setdiff(uint32(1:3), uint32(dDropDimension), 'stable');
dPolygon2d = dPolygonVertices(:, double(ui32KeepDimensions));
dNext2d = dPolygon2d([2:end, 1], :);
dSignedDoubleArea = sum(dPolygon2d(:, 1) .* dNext2d(:, 2) - ...
    dNext2d(:, 1) .* dPolygon2d(:, 2));
dAreaTolerance = max(100.0 .* eps(max(abs(dSignedDoubleArea), 1.0)), ...
    1.0e-14 .* max(dMeshScale .* dMeshScale, 1.0));
if abs(dSignedDoubleArea) <= dAreaTolerance
    error('LoadShapeMesh:BadObjFace', 'OBJ polygon has zero projected area.');
end

dWindingSign = sign(dSignedDoubleArea);

% Remove one valid ear per pass while preserving the source winding.
ui32NumPolygonVertices = uint32(numel(ui32PolygonVertexIds));
ui32Remaining = uint32(1):ui32NumPolygonVertices;
ui32Triangles = zeros(double(ui32NumPolygonVertices - uint32(2)), 3, 'uint32');
ui32TriangleCount = uint32(0);

while numel(ui32Remaining) > 3
    bFoundEar = false;

    for ui32RemainingIdx = uint32(1):uint32(numel(ui32Remaining))
        ui32PreviousIdx = ui32Remaining(mod(double(ui32RemainingIdx) - 2, numel(ui32Remaining)) + 1);
        ui32CurrentIdx = ui32Remaining(double(ui32RemainingIdx));
        ui32NextIdx = ui32Remaining(mod(double(ui32RemainingIdx), numel(ui32Remaining)) + 1);

        dPrevious = dPolygon2d(double(ui32PreviousIdx), :);
        dCurrent = dPolygon2d(double(ui32CurrentIdx), :);
        dNext = dPolygon2d(double(ui32NextIdx), :);

        % A reflex or numerically flat corner cannot be an ear for this winding.
        dEarCross = Cross2d_(dCurrent - dPrevious, dNext - dCurrent);
        if dWindingSign * dEarCross <= dAreaTolerance
            continue
        end

        ui32OtherIndices = setdiff(ui32Remaining, ...
            [ui32PreviousIdx, ui32CurrentIdx, ui32NextIdx], 'stable');
        bContainsOtherVertex = false;

        % An ear is valid only when no other remaining vertex lies inside it.
        for ui32OtherIdx = ui32OtherIndices
            if IsPointInTriangle2d_(dPolygon2d(double(ui32OtherIdx), :), ...
                    dPrevious, dCurrent, dNext, dWindingSign, dAreaTolerance)
                bContainsOtherVertex = true;
                break
            end
        end
        if bContainsOtherVertex
            continue
        end

        % Emit the ear in source-index space, then remove its center from the polygon.
        ui32TriangleCount = ui32TriangleCount + uint32(1);
        ui32Triangles(double(ui32TriangleCount), :) = ui32PolygonVertexIds( ...
            double([ui32PreviousIdx, ui32CurrentIdx, ui32NextIdx]));
        ui32Remaining(double(ui32RemainingIdx)) = [];
        bFoundEar = true;
        break
    end

    if ~bFoundEar
        error('LoadShapeMesh:BadObjFace', ...
            'OBJ polygon must be simple and triangulable.');
    end
end

% The final three vertices form the last triangle without another search pass.
ui32TriangleCount = ui32TriangleCount + uint32(1);
ui32Triangles(double(ui32TriangleCount), :) = ui32PolygonVertexIds(double(ui32Remaining));
end

function dCross = Cross2d_(dVector1, dVector2)
dCross = dVector1(1) .* dVector2(2) - dVector1(2) .* dVector2(1);
end

function bInside = IsPointInTriangle2d_(dPoint, dVertex1, dVertex2, dVertex3, ...
        dWindingSign, dTolerance)
dCross1 = dWindingSign * Cross2d_(dVertex2 - dVertex1, dPoint - dVertex1);
dCross2 = dWindingSign * Cross2d_(dVertex3 - dVertex2, dPoint - dVertex2);
dCross3 = dWindingSign * Cross2d_(dVertex1 - dVertex3, dPoint - dVertex3);
bInside = all([dCross1, dCross2, dCross3] >= -dTolerance);
end

function [dVerticesPos, ui32FaceVertexIds] = ReadStl_(charMeshPath)
%% DESCRIPTION
% Classify STL encoding and dispatch to the matching parser.
% -------------------------------------------------------------------------------------------------------------

strFileInfo = dir(charMeshPath);
dFileSizeBytes = double(strFileInfo.bytes);
[bBinaryStl, bAsciiStl, dExpectedBinaryBytes] = ClassifyStl_( ...
    charMeshPath, dFileSizeBytes);

if bBinaryStl
    [dVerticesPos, ui32FaceVertexIds] = ReadBinaryStl_(charMeshPath);
elseif bAsciiStl
    [dVerticesPos, ui32FaceVertexIds] = ReadAsciiStl_(charMeshPath);
elseif dExpectedBinaryBytes > dFileSizeBytes
    error('LoadShapeMesh:TruncatedBinaryStl', ...
        'Binary STL triangle count requires %.0f bytes, but the file contains %.0f bytes.', ...
        dExpectedBinaryBytes, dFileSizeBytes);
else
    error('LoadShapeMesh:BinaryStlSizeMismatch', ...
        'Binary STL file size does not match its triangle-count header.');
end
end

function [bBinaryStl, bAsciiStl, dExpectedBinaryBytes] = ClassifyStl_( ...
        charMeshPath, dFileSizeBytes)
%% DESCRIPTION
% Prefer an exact binary-size match because a binary header may begin with `solid`.
% -------------------------------------------------------------------------------------------------------------

bBinaryStl = false;
bAsciiStl = false;
dExpectedBinaryBytes = inf;

i32FileId = fopen(charMeshPath, 'r', 'ieee-le');
if i32FileId < 0
    error('LoadShapeMesh:OpenFailed', 'Could not open STL file: %s', charMeshPath);
end
objCleanup = onCleanup(@() fclose(i32FileId));
ui8Prefix = fread(i32FileId, min(84.0, dFileSizeBytes), 'uint8=>uint8');

% A complete binary STL has an exact size implied by its triangle count.
if dFileSizeBytes >= 84.0
    fseek(i32FileId, 80, 'bof');
    ui32NumTriangles = fread(i32FileId, 1, 'uint32=>uint32');
    dExpectedBinaryBytes = 84.0 + 50.0 .* double(ui32NumTriangles);
    bBinaryStl = dExpectedBinaryBytes == dFileSizeBytes;
end

% Interpret `solid` as ASCII only after excluding an exact binary layout.
charPrefix = lower(strtrim(char(transpose(ui8Prefix))));
bAsciiStl = ~bBinaryStl && startsWith(charPrefix, 'solid');

if dFileSizeBytes < 84.0 && ~bAsciiStl
    dExpectedBinaryBytes = 84.0;
end
end

function [dVerticesPos, ui32FaceVertexIds] = ReadAsciiStl_(charMeshPath)
%% DESCRIPTION
% Parse the ASCII STL grammar with explicit facet and loop state.
% -------------------------------------------------------------------------------------------------------------

i32FileId = fopen(charMeshPath, 'r');
if i32FileId < 0
    error('LoadShapeMesh:OpenFailed', 'Could not open ASCII STL file: %s', charMeshPath);
end
objCleanup = onCleanup(@() fclose(i32FileId));

dVerticesTmp = zeros(3000, 3);
ui32NumVertices = uint32(0);
ui32NumFacets = uint32(0);
ui32FacetVertexCount = uint32(0);
bInsideFacet = false;
bInsideLoop = false;

% Track grammar state so malformed ordering fails at the offending record.
while true
    charLine = fgetl(i32FileId);
    if ~ischar(charLine)
        break
    end

    charLine = strtrim(charLine);
    charLowerLine = lower(charLine);

    if isempty(charLine) || strcmp(charLowerLine, 'solid') || ...
            startsWith(charLowerLine, 'solid ')
        continue
    elseif startsWith(charLowerLine, 'facet normal ')
        if bInsideFacet || bInsideLoop
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL contains nested facet declarations.');
        end

        bInsideFacet = true;
        ui32FacetVertexCount = uint32(0);
    elseif strcmp(charLowerLine, 'outer loop')
        if ~bInsideFacet || bInsideLoop
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL outer loop is outside a facet.');
        end

        bInsideLoop = true;
    elseif startsWith(charLowerLine, 'vertex ')
        if ~bInsideFacet || ~bInsideLoop
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL vertex is outside a facet loop.');
        end
        dVertex = sscanf(charLine(7:end), '%f');
        if numel(dVertex) ~= 3 || any(~isfinite(dVertex))
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL vertex must contain three finite coordinates.');
        end

        ui32FacetVertexCount = ui32FacetVertexCount + uint32(1);
        if ui32FacetVertexCount > uint32(3)
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL facets must contain exactly three vertices.');
        end
        ui32NumVertices = ui32NumVertices + uint32(1);
        dVerticesTmp = EnsureDoubleRows_(dVerticesTmp, ui32NumVertices);
        dVerticesTmp(double(ui32NumVertices), :) = transpose(dVertex);
    elseif strcmp(charLowerLine, 'endloop')
        if ~bInsideLoop || ui32FacetVertexCount ~= uint32(3)
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL loop must end after exactly three vertices.');
        end

        bInsideLoop = false;
    elseif strcmp(charLowerLine, 'endfacet')
        if ~bInsideFacet || bInsideLoop || ui32FacetVertexCount ~= uint32(3)
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL facet termination is incomplete or out of order.');
        end

        bInsideFacet = false;
        ui32NumFacets = ui32NumFacets + uint32(1);
    elseif startsWith(charLowerLine, 'endsolid')
        if bInsideFacet || bInsideLoop
            error('LoadShapeMesh:BadAsciiStl', ...
                'ASCII STL ended before closing its final facet.');
        end
    else
        error('LoadShapeMesh:BadAsciiStl', ...
            'Unsupported ASCII STL record: %s', charLine);
    end
end

% A valid file contains one closed three-vertex loop per completed facet.
if bInsideFacet || bInsideLoop || ui32NumFacets == uint32(0) || ...
        double(ui32NumVertices) ~= 3.0 * double(ui32NumFacets)
    error('LoadShapeMesh:BadAsciiStl', 'ASCII STL facet structure is incomplete.');
end

% ASCII STL repeats vertices per facet, so faces map to consecutive triples.
dVerticesPos = dVerticesTmp(1:double(ui32NumVertices), :);
ui32FaceVertexIds = uint32(reshape(1:double(ui32NumVertices), 3, []).');
end

function [dVerticesPos, ui32FaceVertexIds] = ReadBinaryStl_(charMeshPath)
%% DESCRIPTION
% Parse fixed-width little-endian binary STL triangle records.
% -------------------------------------------------------------------------------------------------------------

i32FileId = fopen(charMeshPath, 'r', 'ieee-le');
if i32FileId < 0
    error('LoadShapeMesh:OpenFailed', 'Could not open binary STL file: %s', charMeshPath);
end
objCleanup = onCleanup(@() fclose(i32FileId));

% Skip the descriptive header and allocate one independent vertex triple per face.
fread(i32FileId, 80, 'uint8');
ui32NumTriangles = fread(i32FileId, 1, 'uint32=>uint32');
dVerticesPos = zeros(double(ui32NumTriangles) .* 3, 3);
ui32FaceVertexIds = zeros(double(ui32NumTriangles), 3, 'uint32');

for ui32TriangleIdx = uint32(1):ui32NumTriangles
    % Consume the complete 50-byte record, retaining geometry and validating truncation.
    [~, dNormalValuesRead] = fread(i32FileId, 3, 'float32');
    [dTriangleVertices, dVertexValuesRead] = fread(i32FileId, [3, 3], 'float32=>double');
    [~, dAttributeValuesRead] = fread(i32FileId, 1, 'uint16');
    if dNormalValuesRead ~= 3.0 || dVertexValuesRead ~= 9.0 || dAttributeValuesRead ~= 1.0
        error('LoadShapeMesh:TruncatedBinaryStl', ...
            'Binary STL triangle record %u is incomplete.', ui32TriangleIdx);
    end

    % Preserve face order by assigning each record to a consecutive vertex triple.
    ui32VertexStart = (ui32TriangleIdx - uint32(1)) .* uint32(3) + uint32(1);
    ui32VertexIds = ui32VertexStart:ui32VertexStart + uint32(2);
    dVerticesPos(double(ui32VertexIds), :) = transpose(dTriangleVertices);
    ui32FaceVertexIds(double(ui32TriangleIdx), :) = ui32VertexIds;
end
end

function [dVerticesPos, ui32FaceVertexIds] = RepairMesh_( ...
        dVerticesPos, ui32FaceVertexIds, charMeshPath)
%% DESCRIPTION
% Weld exact duplicates, reject degenerate faces, and compact unreferenced vertices.
% -------------------------------------------------------------------------------------------------------------

% Remap all faces through one deterministic exact-coordinate vertex set.
[dVerticesPos, ~, dVertexMap] = unique(dVerticesPos, 'rows');
ui32FaceVertexIds = reshape(uint32(dVertexMap(double(ui32FaceVertexIds))), ...
    size(ui32FaceVertexIds));

% Remove repeated-index and near-zero-area triangles using a mesh-scaled tolerance.
dMeshExtent = max(max(dVerticesPos, [], 1) - min(dVerticesPos, [], 1));
dAreaScale = max(dMeshExtent .* dMeshExtent, realmin);
dAreaTolerance = 100.0 .* eps(dAreaScale);

dVertex1 = dVerticesPos(double(ui32FaceVertexIds(:, 1)), :);
dVertex2 = dVerticesPos(double(ui32FaceVertexIds(:, 2)), :);
dVertex3 = dVerticesPos(double(ui32FaceVertexIds(:, 3)), :);

bDistinctIndices = ui32FaceVertexIds(:, 1) ~= ui32FaceVertexIds(:, 2) & ...
    ui32FaceVertexIds(:, 1) ~= ui32FaceVertexIds(:, 3) & ...
    ui32FaceVertexIds(:, 2) ~= ui32FaceVertexIds(:, 3);
dDoubleArea = vecnorm(cross(dVertex2 - dVertex1, dVertex3 - dVertex1, 2), 2, 2);
ui32FaceVertexIds = ui32FaceVertexIds(bDistinctIndices & dDoubleArea > dAreaTolerance, :);

if isempty(ui32FaceVertexIds)
    error('LoadShapeMesh:DegenerateMesh', ...
        'Shape mesh has no nondegenerate triangles: %s', charMeshPath);
end

% Compact the surviving vertex set and rewrite faces to contiguous one-based indices.
ui32UsedVertices = unique(ui32FaceVertexIds(:));
ui32VertexMap = zeros(size(dVerticesPos, 1), 1, 'uint32');
ui32VertexMap(double(ui32UsedVertices)) = uint32(1):uint32(numel(ui32UsedVertices));
ui32FaceVertexIds = reshape(ui32VertexMap(double(ui32FaceVertexIds)), ...
    size(ui32FaceVertexIds));
dVerticesPos = dVerticesPos(double(ui32UsedVertices), :);
end

function ValidateMesh_(dVerticesPos, ui32FaceVertexIds, charMeshPath)
%% DESCRIPTION
% Enforce the format-independent geometry contract before optional repair.
% -------------------------------------------------------------------------------------------------------------

if isempty(dVerticesPos) || isempty(ui32FaceVertexIds)
    error('LoadShapeMesh:EmptyMesh', ...
        'Shape mesh has no usable triangles: %s', charMeshPath);
end

if any(~isfinite(dVerticesPos), 'all')
    error('LoadShapeMesh:BadVertex', ...
        'Shape mesh contains non-finite vertex coordinates: %s', charMeshPath);
end

if max(ui32FaceVertexIds, [], 'all') > uint32(size(dVerticesPos, 1))
    error('LoadShapeMesh:BadFaceIndex', ...
        'Shape mesh face index exceeds vertex count: %s', charMeshPath);
end
end

function dRows = EnsureDoubleRows_(dRows, ui32RequiredRows)
if ui32RequiredRows <= uint32(size(dRows, 1))
    return
end

% Double capacity so record-wise parsing remains amortized linear.
dRows(end + 1:double(ui32RequiredRows) * 2, :) = 0.0;
end

function ui32Rows = EnsureUintRows_(ui32Rows, ui32RequiredRows)
if ui32RequiredRows <= uint32(size(ui32Rows, 1))
    return
end

% Double capacity so polygon triangulation does not reallocate for every face.
ui32Rows(end + 1:double(ui32RequiredRows) * 2, :) = uint32(0);
end
