function [dFaceStarts, dFaceEnds, bKeepFaces] = ...
        SelectObjFaceRecords(charFileText, dFaceStarts, dFaceEnds, charObjectNames)
%% SIGNATURE
% [dFaceStarts, dFaceEnds, bKeepFaces] = ...
%     SelectObjFaceRecords(charFileText, dFaceStarts, dFaceEnds, charObjectNames)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Select exact, case-sensitive OBJ object names before decoding face payloads. An empty selection
% keeps every face. The empty string selects unnamed faces, including faces before the first o
% record. Repeated object declarations are combined in source order; g/usemtl never change the
% object. Every requested name must have at least one face. No file, vertex or auxiliary array is
% rewritten. This host-only helper is shared by the two existing vectorized OBJ readers.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charFileText      Complete source text.
% dFaceStarts       Sorted face-record start offsets from regexp, one-based.
% dFaceEnds         Matching face-record end offsets.
% charObjectNames   Exact names; strings(1,0) keeps all objects, "" selects the unnamed object.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dFaceStarts       Selected start offsets, retaining source order.
% dFaceEnds         Matching selected end offsets.
% bKeepFaces        Optional logical mask in the original face-record order.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-09-2026  Pietro Califano     Add shared pre-decode OBJ object selection.
% 29-09-2026  Pietro Califano, Codex gpt-6    Clarify range selection and scratch storage.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% Base MATLAB text and array operations.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    charFileText (1,:) char
    dFaceStarts (1,:) double
    dFaceEnds (1,:) double
    charObjectNames (1,:) string {mustBeNonmissing} = strings(1, 0)
end
arguments (Output)
    dFaceStarts (1,:) double
    dFaceEnds (1,:) double
    bKeepFaces (1,:) logical
end

% Avoid allocating a full mask for unselected loads unless the caller requests it.
if isempty(charObjectNames)
    bKeepFaces = true(1, 0);
    if nargout > 2
        bKeepFaces = true(size(dFaceStarts));
    end
    return
end

% Partition source text by object declarations, including its initial unnamed range.
[dObjectStarts, cellObjectLines] = regexp(charFileText, ...
    '^[ \t]*o(?:[ \t][^\r\n]*)?\r?$', 'start', 'match', 'lineanchors');
charNames = strtrim(string(regexprep(regexprep(cellObjectLines, ...
    '^[ \t]*o[ \t]*', ''), '#.*$', '')));
charNames = ["", charNames];
dRangeStarts = [1, dObjectStarts];
dRangeEnds = [dObjectStarts, numel(charFileText) + 1];
bSelectedRanges = ismember(charNames, charObjectNames);
bKeepFaces = false(size(dFaceStarts));
bFoundNames = false(size(charObjectNames));

% Locate each selected range by binary search instead of rescanning all faces per object.
% Allocate one logical per face plus object descriptors, without per-face strings or object IDs.
for dRangeIndex = find(bSelectedRanges)
    dFirstFace = LowerBound_(dFaceStarts, dRangeStarts(dRangeIndex));
    dLastFace = LowerBound_(dFaceStarts, dRangeEnds(dRangeIndex)) - 1;
    if dFirstFace <= dLastFace
        bKeepFaces(dFirstFace:dLastFace) = true;
        bFoundNames(charObjectNames == charNames(dRangeIndex)) = true;
    end
end

% Reject missing members of a requested union before returning any partial selection.
if ~all(bFoundNames)
    error('SelectObjFaceRecords:MissingObjects', ...
        'Requested OBJ objects have no face records: %s', ...
        strjoin('"' + charObjectNames(~bFoundNames) + '"', ', '));
end

% Preserve source ordering independently of the requested name order.
dFaceStarts = dFaceStarts(bKeepFaces);
dFaceEnds = dFaceEnds(bKeepFaces);
end

function dIndex = LowerBound_(dSortedOffsets, dOffset)
% Return the first offset at least dOffset, or numel+1 when no offset qualifies.
arguments (Input)
    dSortedOffsets (1,:) double
    dOffset (1,1) double
end
arguments (Output)
    dIndex (1,1) double
end

dFirst = 1;
dAfterLast = numel(dSortedOffsets) + 1;
while dFirst < dAfterLast
    dMiddle = floor((dFirst + dAfterLast) / 2);
    if dSortedOffsets(dMiddle) < dOffset
        dFirst = dMiddle + 1;
    else
        dAfterLast = dMiddle;
    end
end
dIndex = dFirst;
end
