function strRecords = NormalizeManifestStructArray(varRecords)
%% SIGNATURE
% strRecords = NormalizeManifestStructArray(varRecords)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Convert JSON-decoded records into a struct array. Missing optional fields are represented by empty arrays so
% callers can inspect heterogeneous manifest records through one stable MATLAB interface.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% varRecords    Struct array or cell array of scalar structs decoded from JSON.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strRecords    Struct array whose records share the union of input fields.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 21-09-2026  Pietro Califano, Codex gpt-5.6  First implementation.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments
    varRecords {mustBeA(varRecords, ["struct", "cell"])}
end

if isstruct(varRecords)
    strRecords = varRecords;
    return
end

if isempty(varRecords)
    strRecords = struct.empty(1, 0);
    return
end

% Build one field set before concatenation because jsondecode returns cells
% when records contain different optional metadata.
cellFieldNames = cellfun(@fieldnames, varRecords, UniformOutput=false);
cellAllFieldNames = unique(vertcat(cellFieldNames{:}), "stable");

cellNormalizedRecords = varRecords;
for dRecordIdx = 1:numel(cellNormalizedRecords)
    for dFieldIdx = 1:numel(cellAllFieldNames)
        charFieldName = cellAllFieldNames{dFieldIdx};
        if ~isfield(cellNormalizedRecords{dRecordIdx}, charFieldName)
            cellNormalizedRecords{dRecordIdx}.(charFieldName) = [];
        end
    end
    cellNormalizedRecords{dRecordIdx} = orderfields( ...
        cellNormalizedRecords{dRecordIdx}, cellAllFieldNames);
end

strRecords = reshape([cellNormalizedRecords{:}], size(varRecords));
end
