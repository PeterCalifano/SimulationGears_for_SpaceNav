function ValidateTriangleRayData(strRayData)
%% SIGNATURE
% ValidateTriangleRayData(strRayData)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Validate a prepared numeric ray payload before saving or compiling it.
% Check fixed capacities, source permutation, tree reachability and conservative
% bounds. Keep this host-only validation outside repeated numerical queries.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strRayData  Payload from BuildTriangleRayData.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; raise an identified error for an invalid payload.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    strRayData (1, 1) struct
end

% Reject ambiguous schemas and nonfinite geometry before inspecting topology.
cellFields = {'ui32TriangleCount', 'dVertex0', 'dEdge1', 'dEdge2', 'dNodeMin', 'dNodeMax', ...
    'ui32NodeLeft', 'ui32NodeRight', 'ui32LeafStart', 'ui32LeafCount', ...
    'ui32TriangleOrder', 'ui32NodeCount', 'bUseBvh'};
assert(isequal(sort(fieldnames(strRayData)), sort(cellFields.')), ...
    'ValidateTriangleRayData:Schema', 'Supply the exact prepared ray schema.');

% Check primitive array types before using indices or node topology.
cellDoubleFields = {'dVertex0', 'dEdge1', 'dEdge2', 'dNodeMin', 'dNodeMax'};
for ui32Field = uint32(1):uint32(numel(cellDoubleFields))
    dValues = strRayData.(cellDoubleFields{ui32Field});
    assert(isa(dValues, 'double') && size(dValues, 1) == 3 && ismatrix(dValues) && ...
        all(isfinite(dValues), 'all'), 'ValidateTriangleRayData:Geometry', 'Use finite 3-by-N doubles.');
end
cellIntegerFields = {'ui32TriangleCount', 'ui32NodeLeft', 'ui32NodeRight', ...
    'ui32LeafStart', 'ui32LeafCount', 'ui32TriangleOrder', 'ui32NodeCount'};
for ui32Field = uint32(1):uint32(numel(cellIntegerFields))
    assert(isa(strRayData.(cellIntegerFields{ui32Field}), 'uint32'), ...
        'ValidateTriangleRayData:IndexType', 'Use uint32 active counts and indices.');
end
assert(isscalar(strRayData.ui32TriangleCount) && isscalar(strRayData.ui32NodeCount) && ...
    islogical(strRayData.bUseBvh) && isscalar(strRayData.bUseBvh), ...
    'ValidateTriangleRayData:Counts', 'Use scalar active counts and traversal selection.');

% Verify active triangle storage and its one-to-one source permutation.
ui32Count = strRayData.ui32TriangleCount;
assert(ui32Count <= size(strRayData.dVertex0, 2) && ...
    isequal(size(strRayData.dVertex0), size(strRayData.dEdge1), size(strRayData.dEdge2)), ...
    'ValidateTriangleRayData:Capacity', 'Triangle data must share sufficient capacity.');
assert(size(strRayData.ui32TriangleOrder, 1) == 1 && ...
    numel(strRayData.ui32TriangleOrder) >= ui32Count && ...
    isequal(sort(strRayData.ui32TriangleOrder(1:ui32Count)), uint32(1:double(ui32Count))), ...
    'ValidateTriangleRayData:Permutation', 'Preserve every active source triangle exactly once.');

% Verify shared node capacity before traversing the rooted tree.
ui32NodeCount = strRayData.ui32NodeCount;
dNodeCapacity = size(strRayData.dNodeMin, 2);
assert(isequal(size(strRayData.dNodeMin), size(strRayData.dNodeMax)) && ...
    ui32NodeCount <= dNodeCapacity && ...
    all(cellfun(@(charField) isequal(size(strRayData.(charField)), [1, dNodeCapacity]), ...
        {'ui32NodeLeft', 'ui32NodeRight', 'ui32LeafStart', 'ui32LeafCount'})), ...
    'ValidateTriangleRayData:NodeCapacity', 'Node arrays must share sufficient capacity.');
if ui32NodeCount == 0
    assert(~strRayData.bUseBvh || ui32Count == 0, ...
        'ValidateTriangleRayData:MissingTree', 'Prepare a tree before enabling BVH traversal.');
    return
end

% Require a rooted acyclic tree within the balanced traversal stack bound.
ui32ParentCount = zeros(1, ui32NodeCount, 'uint32');
ui32Depth = zeros(1, ui32NodeCount, 'uint32');
ui32Depth(1) = 1;
ui32Coverage = zeros(1, ui32Count, 'uint32');

for ui32Node = uint32(1):ui32NodeCount
    assert(all(strRayData.dNodeMin(:, ui32Node) <= strRayData.dNodeMax(:, ui32Node)), ...
        'ValidateTriangleRayData:Bounds', 'Bounds must not be inverted.');

    if strRayData.ui32LeafCount(ui32Node)>0
        ui32Start = strRayData.ui32LeafStart(ui32Node);
        ui32End = ui32Start + strRayData.ui32LeafCount(ui32Node) - 1;
        assert(ui32Start >= 1 && ui32End <= ui32Count, ...
            'ValidateTriangleRayData:Leaf', 'Leaves must reference the active permutation.');
        ui32Ids = strRayData.ui32TriangleOrder(ui32Start:ui32End);
        ui32Coverage(ui32Ids) = ui32Coverage(ui32Ids) + 1;
        dVertices = cat(3, strRayData.dVertex0(:, ui32Ids), ...
            strRayData.dVertex0(:, ui32Ids) + strRayData.dEdge1(:, ui32Ids), ...
            strRayData.dVertex0(:, ui32Ids) + strRayData.dEdge2(:, ui32Ids));
        assert(all(dVertices >= strRayData.dNodeMin(:, ui32Node), 'all') && ...
            all(dVertices <= strRayData.dNodeMax(:, ui32Node), 'all'), ...
            'ValidateTriangleRayData:Bounds', 'Leaf bounds must contain complete triangles.');
    else
        ui32Children = [strRayData.ui32NodeLeft(ui32Node), strRayData.ui32NodeRight(ui32Node)];
        assert(all(ui32Children>ui32Node & ui32Children <= ui32NodeCount) && ...
            ui32Children(1) ~= ui32Children(2), ...
            'ValidateTriangleRayData:Topology', 'Children must follow their unique parent.');
        ui32ParentCount(ui32Children) = ui32ParentCount(ui32Children) + 1;
        ui32Depth(ui32Children) = ui32Depth(ui32Node) + 1;
        assert(all(strRayData.dNodeMin(:, ui32Children) >= strRayData.dNodeMin(:, ui32Node), 'all') && ...
            all(strRayData.dNodeMax(:, ui32Children) <= strRayData.dNodeMax(:, ui32Node), 'all'), ...
            'ValidateTriangleRayData:Bounds', 'Parent bounds must contain child bounds.');
    end
end

% Every active triangle and non-root node must be owned exactly once.
assert(ui32ParentCount(1) == 0 && all(ui32ParentCount(2:end) == 1) && all(ui32Coverage == 1) && ...
    max(ui32Depth) <= min(34, ceil(log2(max(1, size(strRayData.dVertex0, 2)))) + 2), ...
    'ValidateTriangleRayData:Topology', 'Require complete coverage and bounded tree depth.');
end
