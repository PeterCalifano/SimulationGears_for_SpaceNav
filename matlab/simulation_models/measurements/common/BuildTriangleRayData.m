function strRayData = BuildTriangleRayData(dFaceVertices, bUseBvh)
%% SIGNATURE
% strRayData = BuildTriangleRayData(dFaceVertices, bUseBvh)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Prepare static triangle edges and an optional balanced binary BVH.
% Adapt the median-partition builder from RCS-1 BuildLidarBvh (9b6a2d47).
% Keep original one-based triangle identities. Store a flat numeric payload in
% one geometry length unit; freeze array capacities when deriving codegen types.
% Rebuild after changing vertices, topology or units. Rigid frame motion needs
% only a transformed ray. Use the runtime selector without freezing mesh values.
% Example: strData = BuildTriangleRayData(reshape([0;0;0;1;0;0;0;1;0],3,3,1), true);
% Output: One triangle with precomputed edges and a single BVH leaf.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dFaceVertices    (3,3,N) triangle vertices [length].
% bUseBvh          Build and select BVH traversal; false selects a flat scan.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strRayData       Fixed-shape numeric geometry and traversal data.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% 07-10-2026  Pietro Califano, Codex (GPT-6)  Generalize the RCS-1 builder with compact numeric storage.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None; host-side geometry preparation.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dFaceVertices (3, 3, :) double {mustBeFinite}
    bUseBvh (1, 1) logical = false
end

arguments (Output)
    strRayData (1, 1) struct
end

% Preserve source identities independently of the traversal permutation.
ui32TriangleCount = uint32(size(dFaceVertices, 3));
dVertex0 = reshape(dFaceVertices(:, 1, :), 3, []);
dEdge1 = reshape(dFaceVertices(:, 2, :), 3, []) - dVertex0;
dEdge2 = reshape(dFaceVertices(:, 3, :), 3, []) - dVertex0;
dTriangleMin = reshape(min(dFaceVertices, [], 2), 3, []);
dTriangleMax = reshape(max(dFaceVertices, [], 2), 3, []);

% Reserve fixed capacities and keep validation outside repeated ray queries.
ui32NodeCapacity = uint32(1);
if bUseBvh && ui32TriangleCount > 0
    ui32NodeCapacity = uint32(2 * 2 ^ ceil(log2(max(1, double(ui32TriangleCount) / 8))) - 1);
end
strRayData = struct( ...
    'ui32TriangleCount', ui32TriangleCount, ...
    'dVertex0', dVertex0, ...
    'dEdge1', dEdge1, ...
    'dEdge2', dEdge2, ...
    'dNodeMin', zeros(3, ui32NodeCapacity), ...
    'dNodeMax', zeros(3, ui32NodeCapacity), ...
    'ui32NodeLeft', zeros(1, ui32NodeCapacity, 'uint32'), ...
    'ui32NodeRight', zeros(1, ui32NodeCapacity, 'uint32'), ...
    'ui32LeafStart', zeros(1, ui32NodeCapacity, 'uint32'), ...
    'ui32LeafCount', zeros(1, ui32NodeCapacity, 'uint32'), ...
    'ui32TriangleOrder', uint32(1:double(ui32TriangleCount)), ...
    'ui32NodeCount', uint32(0), ...
    'bUseBvh', bUseBvh);
if ~bUseBvh || ui32TriangleCount == 0
    return
end

% Split by centroid while bounding the complete triangle extent conservatively.
dCentres = 0.5 * dTriangleMin + 0.5 * dTriangleMax;
dBoundsPadding = 64 * eps * max(1, max(abs(dFaceVertices), [], [2, 3]));
ui32PendingStart = zeros(1, ui32NodeCapacity, 'uint32');
ui32PendingCount = zeros(1, ui32NodeCapacity, 'uint32');
ui32PendingStart(1) = 1;
ui32PendingCount(1) = ui32TriangleCount;
strRayData.ui32NodeCount = uint32(1);
ui32Node = uint32(1);

while ui32Node <= strRayData.ui32NodeCount
    
    ui32Start = ui32PendingStart(ui32Node);
    ui32Count = ui32PendingCount(ui32Node);
    ui32End = ui32Start + ui32Count - 1;
    ui32Indices = strRayData.ui32TriangleOrder(ui32Start:ui32End);
    strRayData.dNodeMin(:, ui32Node) = min(dTriangleMin(:, ui32Indices), [], 2) - dBoundsPadding;
    strRayData.dNodeMax(:, ui32Node) = max(dTriangleMax(:, ui32Indices), [], 2) + dBoundsPadding;

    if ui32Count <= 8
        strRayData.ui32LeafStart(ui32Node) = ui32Start;
        strRayData.ui32LeafCount(ui32Node) = ui32Count;
    else
        % Partition source IDs in place with alanced median split
        [~, dAxis] = max(strRayData.dNodeMax(:, ui32Node) - strRayData.dNodeMin(:, ui32Node));
        ui32LeftCount = idivide(ui32Count, uint32(2), 'floor');
        ui32Median = ui32Start + ui32LeftCount - 1;
        ui32Lower = ui32Start;
        ui32Upper = ui32End;

        while ui32Lower < ui32Upper
        
            ui32PivotIndex = idivide(ui32Lower + ui32Upper, uint32(2), 'floor');
            ui32PivotId = strRayData.ui32TriangleOrder(ui32PivotIndex);
            dPivot = dCentres(dAxis, ui32PivotId);
            ui32First = ui32Lower;
            ui32Last = ui32Upper;

            while ui32First <= ui32Last
                while ui32First <= ui32Upper && ...
                        (dCentres(dAxis, strRayData.ui32TriangleOrder(ui32First)) < dPivot || ...
                        (dCentres(dAxis, strRayData.ui32TriangleOrder(ui32First)) == dPivot && ...
                        strRayData.ui32TriangleOrder(ui32First) < ui32PivotId))
                    ui32First = ui32First + 1;
                end
                while ui32Last >= ui32Lower && ...
                        (dCentres(dAxis, strRayData.ui32TriangleOrder(ui32Last)) > dPivot || ...
                        (dCentres(dAxis, strRayData.ui32TriangleOrder(ui32Last)) == dPivot && ...
                        strRayData.ui32TriangleOrder(ui32Last) > ui32PivotId))
                    ui32Last = ui32Last - 1;
                end
                if ui32First <= ui32Last
                    ui32Tmp = strRayData.ui32TriangleOrder(ui32First);
                    strRayData.ui32TriangleOrder(ui32First) = strRayData.ui32TriangleOrder(ui32Last);
                    strRayData.ui32TriangleOrder(ui32Last) = ui32Tmp;
                    ui32First = ui32First + 1;
                    ui32Last = ui32Last - 1;
                end
            end

            if ui32Median <= ui32Last
                ui32Upper = ui32Last;
            elseif ui32Median >= ui32First
                ui32Lower = ui32First;
            else
                break
            end
        end
        
        ui32Left = strRayData.ui32NodeCount + 1;
        ui32Right = ui32Left + 1;
        strRayData.ui32NodeLeft(ui32Node) = ui32Left;
        strRayData.ui32NodeRight(ui32Node) = ui32Right;
        ui32PendingStart(ui32Left) = ui32Start;
        ui32PendingStart(ui32Right) = ui32Start + ui32LeftCount;
        ui32PendingCount(ui32Left) = ui32LeftCount;
        ui32PendingCount(ui32Right) = ui32Count - ui32LeftCount;
        strRayData.ui32NodeCount = ui32Right;
    end
    ui32Node = ui32Node + 1;
end

% Keep the deployed tree compact; code generation freezes these prepared sizes.
ui32ActiveNodes = uint32(1):strRayData.ui32NodeCount;
strRayData.dNodeMin = strRayData.dNodeMin(:, ui32ActiveNodes);
strRayData.dNodeMax = strRayData.dNodeMax(:, ui32ActiveNodes);
strRayData.ui32NodeLeft = strRayData.ui32NodeLeft(ui32ActiveNodes);
strRayData.ui32NodeRight = strRayData.ui32NodeRight(ui32ActiveNodes);
strRayData.ui32LeafStart = strRayData.ui32LeafStart(ui32ActiveNodes);
strRayData.ui32LeafCount = strRayData.ui32LeafCount(ui32ActiveNodes);
end
