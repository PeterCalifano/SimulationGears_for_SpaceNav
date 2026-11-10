function strResults = testPreparedTriangleTracing()
%% SIGNATURE
% strResults = testPreparedTriangleTracing()
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Check analytical hits, nearest-hit ordering, ray intervals and exact BVH parity.
% Use deterministic synthetic meshes without production assets. Preserve the
% caller's random stream. Exercise the actual mesh and LiDAR model interfaces.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% None; add SimulationGears and MathCore through their normal setup.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% strResults  Analytical/randomized case counts and maximum parity discrepancy.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% 07-10-2026  Pietro Califano, Codex (GPT-6)  Check sensor misses, inclusive bounds and cache serialization.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% BuildTriangleRayData, TraceTriangleRay, RayTraceTriangMesh, LaserRangefinderModel.
% CShapeModel, CBaseDatastruct, yaml (load the repository's lib/yaml provider).
% -------------------------------------------------------------------------------------------------------------

arguments (Output)
    strResults (1, 1) struct
end

% Place a nearer surface after a farther record and beyond the nominal centre.
dTriangle = [0, 1, 0; 0, 0, 1; 0, 0, 0];
dFaces = cat(3, dTriangle + [0;0;5], dTriangle + [0;0;2], dTriangle - [0;0;1]);
strBvh = BuildTriangleRayData(dFaces, true);
strFlat = strBvh;
strFlat.bUseBvh = false;
strQuery = struct('dDirection', [0;0;1], 'bAnyHit', false, 'bTwoSided', true, ...
    'dMinDistance', 0, 'dMaxDistance', Inf, 'ui32IgnoreTriangle', uint32(0));
[bHit, dRange, dPoint, ui32Id] = TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery);
assert(bHit && dRange == 2 && ui32Id == 2 && norm(dPoint-[0.2;0.2;2]) < 1e-14);

% Require the same physical return under every triangle-record permutation.
dPermutations = perms(1:3);
for ui32Index = uint32(1):uint32(size(dPermutations, 1))
    strData = BuildTriangleRayData(dFaces(:, :, dPermutations(ui32Index, :)), true);
    [bHit, dRange] = TraceTriangleRay(strData, [0.2;0.2;0], strQuery);
    assert(bHit && dRange == 2);
end

% Check open lower bounds, closed upper bounds, exclusions and one-sided tests.
strQuery.dMaxDistance = 1.9;
assert(~TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery));
strQuery.dMaxDistance = 2;
assert(TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery));
strQuery.dMaxDistance = Inf;
strQuery.dMinDistance = 2;
[bHit, dRange] = TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery);
assert(bHit && dRange == 5);
strQuery.dMinDistance = 0;
strQuery.ui32IgnoreTriangle = uint32(2);
[bHit, dRange] = TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery);
assert(bHit && dRange == 5);
strQuery.ui32IgnoreTriangle = uint32(0);
strQuery.bTwoSided = false;
assert(~TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery));
strQuery.bTwoSided = true;
strQuery.dDirection = [0;0;2];
[bHit, dRange, dPoint] = TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery);
assert(bHit && dRange == 1 && norm(dPoint-[0.2;0.2;2]) < 1e-14);
strQuery.dDirection = [0;0;1];

% Reject empty/degenerate meshes and preserve exact nearest-hit ties.
strEmpty = BuildTriangleRayData(zeros(3, 3, 0), true);
assert(~TraceTriangleRay(strEmpty, zeros(3, 1), strQuery));
strDegenerate = BuildTriangleRayData(zeros(3, 3, 1), true);
assert(~TraceTriangleRay(strDegenerate, zeros(3, 1), strQuery));
strTie = BuildTriangleRayData(repmat(dFaces(:, :, 2), 1, 1, 9), true);
[~, ~, ~, ui32Id] = TraceTriangleRay(strTie, [0.2;0.2;0], strQuery);
assert(ui32Id == 1);

% Exercise source sensor calls with the prepared and legacy mesh schemas.
strMesh = struct('dVerticesPositions', reshape(dFaces, 3, []), ...
    'i32triangVertexPtrs', reshape(int32(1:9), 3, []));
[bHit, dRange] = RayTraceTriangMesh(strMesh, [0;0;1], [0.2;0.2;0], [0;0;1], false);
assert(bHit && dRange == 2);
strMesh.strRayData = strBvh;
[dRange, bHit, bValid] = LaserRangefinderModel(strMesh, [0;0;1], [0.2;0.2;0], ...
    0, 0, [0;0;1], [0;10], false, false, true);
assert(bHit && bValid && dRange == 2);
[~, bHit, bValid] = LaserRangefinderModel(strMesh, [0;0;1], [0.2;0.2;0], ...
    0, 0, [0;0;1], [0;1], false, false, true);
assert(bHit && ~bValid);

% Accept a visible protrusion when the nominal centre is behind the sensor.
[dRange, bHit, bValid] = LaserRangefinderModel(strMesh, [0;0;1], [0.2;0.2;0], ...
    0, 0, [0;0;-10], [0;10], false, false, true);
assert(bHit && bValid && dRange == 2);
[dRange, bHit, bValid] = LaserRangefinderModel(strMesh, [0;0;-1], [0.2;0.2;-2], ...
    0, 0, zeros(3, 1), [0;10], false, false, false);
assert(~bHit && ~bValid && dRange == -1);

% A geometric miss consumes no sensor noise and retains exact miss outputs.
strRngBeforeMiss = rng;
[dRange, bHit, bValid, dPoint, dError] = LaserRangefinderModel( ...
    strMesh, [0;0;-1], [0.2;0.2;-2], 100, 100, zeros(3, 1), [0;10], true, false, false);
assert(~bHit && ~bValid && dRange == -1 && dError == 0 && all(dPoint == 0));
assert(isequal(rng, strRngBeforeMiss));

% Include both range endpoints after sensor bias is applied.
for dBound = [2, 3]
    [dRange, bHit, bValid, ~, dError] = LaserRangefinderModel( ...
        strMesh, [0;0;1], [0.2;0.2;0], 0, dBound - 2, zeros(3, 1), ...
        [dBound;dBound], true, false, true);
    assert(bHit && bValid && dRange == dBound && dError == dBound - 2);
end

% Check grazing/shared-edge hits and nearly parallel rays on analytical geometry.
for dOrigin = [0, 1, 0.5; 0, 0, 0.5; 0, 0, 0]
    [bHit, dRange] = TraceTriangleRay(strBvh, dOrigin, strQuery);
    assert(bHit && dRange == 2);
end
strQuery.dDirection = [1;0;1e-18];
assert(~TraceTriangleRay(strBvh, [0.2;0.2;0], strQuery));
strQuery.dDirection = [0;0;1];

% Reject malformed host payloads before they reach unchecked generated queries.
cellInvalid = cell(1, 10);
cellInvalid{1} = rmfield(strBvh, 'dEdge1');
cellInvalid{2} = strBvh;
cellInvalid{2}.dVertex0(1) = NaN;
cellInvalid{3} = strBvh;
cellInvalid{3}.ui32TriangleCount = 3;
cellInvalid{4} = strBvh;
cellInvalid{4}.ui32TriangleOrder(1) = 2;
cellInvalid{5} = strBvh;
cellInvalid{5}.ui32TriangleCount = uint32(20);
cellInvalid{6} = strBvh;
cellInvalid{6}.dNodeMax(:, 1) = -Inf;
cellInvalid{7} = strBvh;
cellInvalid{7}.ui32LeafStart(1) = 0;
cellInvalid{8} = strBvh;
cellInvalid{8}.dNodeMin(:, 1) = [100;100;100];
cellInvalid{9} = strBvh;
cellInvalid{9}.ui32NodeCount = uint32(0);
cellInvalid{10} = strBvh;
cellInvalid{10}.bUseBvh = 1;
for ui32Case = uint32(1):uint32(numel(cellInvalid))
    ExpectInvalidRayData_(cellInvalid{ui32Case});
end

% Cache numeric geometry once and invalidate it through existing mesh mutation.
objShape = CShapeModel('struct', struct('ui32triangVertexPtr', uint32(reshape(1:9, 3, [])), ...
    'dVerticesPos', reshape(dFaces, 3, [])), 'm', 'm', true, 'Fixture', true);
objShape = objShape.prepareRayTracingData(true);
strCached = objShape.getRayTracingModel();
assert(isfield(strCached, 'strRayData') && strCached.strRayData.ui32TriangleCount == 3);
objShape = objShape.prepareRayTracingData(true);
assert(isequal(objShape.getRayTracingModel(), strCached));

% Runtime caches are excluded from direct, nested, JSON, YAML and MAT exports.
strExport = objShape.toStruct();
assert(~isfield(strExport, 'strRayCache_') && isfield(strExport, 'dVerticesPos'));
strNested = CBaseDatastruct.toStructStatic(struct('objShape', objShape));
assert(~isfield(strNested.objShape, 'strRayCache_'));
assert(~contains(CBaseDatastruct.toJsonStatic(objShape), 'strRayCache_'));
assert(~contains(CBaseDatastruct.toYamlStatic(objShape), 'strRayCache_'));
charSavedShape = [tempname, '.mat'];
objSaveCleanup = onCleanup(@() delete(charSavedShape)); %#ok<NASGU>
save(charSavedShape, 'objShape');
strLoaded = load(charSavedShape, 'objShape');
assert(~isfield(strLoaded.objShape.getRayTracingModel(), 'strRayData'));
assert(isequal(strLoaded.objShape.getShapeStruct(), objShape.getShapeStruct()));

objShape.charTargetUnitOutput = 'km';
try
    objShape.getRayTracingModel();
    error('testPreparedTriangleTracing:MissingUnitError', 'Stale units were accepted.');
catch objError
    assert(strcmp(objError.identifier, 'CShapeModel:StaleRayUnits'));
end
objShape.charTargetUnitOutput = 'm';
objShape = objShape.SimplifyMesh(100);
strCleared = objShape.getRayTracingModel();
assert(~isfield(strCleared, 'strRayData') && isempty(strCleared.dVerticesPositions));

% Compare random queries against the flat oracle in identical transformed units.
strInitialRng = rng;
objRandomCleanup = onCleanup(@() rng(strInitialRng)); %#ok<NASGU>
rng(58031, 'twister');
dRandomFaces = randn(3, 3, 96);
dOrigins = 3*randn(3, 512);
dDirections = randn(3, 512);
dMaxError = 0;
for dScale = [1e-3, 1, 1e3]
    dShift = [17;-9;3]*dScale;
    strBvh = BuildTriangleRayData(dScale*dRandomFaces+dShift, true);
    strFlat = strBvh;
    strFlat.bUseBvh = false;
    for ui32Ray = uint32(1):uint32(size(dDirections, 2))
        strQuery.dDirection = dDirections(:, ui32Ray);
        dOrigin = dScale*dOrigins(:, ui32Ray)+dShift;
        [bBvh, dBvh, dBvhPoint, ui32BvhId] = TraceTriangleRay(strBvh, dOrigin, strQuery);
        [bFlat, dFlat, dFlatPoint, ui32FlatId] = TraceTriangleRay(strFlat, dOrigin, strQuery);
        assert(bBvh == bFlat && ui32BvhId == ui32FlatId);
        assert(abs(dBvh-dFlat) <= 1e-12*max(1, abs(dFlat)));
        dMaxError = max(dMaxError, norm(dBvhPoint-dFlatPoint));
        strQuery.bAnyHit = true;
        assert(TraceTriangleRay(strBvh, dOrigin, strQuery) == TraceTriangleRay(strFlat, dOrigin, strQuery));
        strQuery.bAnyHit = false;
    end
end
strResults = struct('ui32PayloadRejections', uint32(10), ...
    'ui32RandomizedCases', uint32(3072), 'dMaxPositionDifference', dMaxError);
fprintf('PREPARED_TRACE_PASS: %u random mode/unit cases, maximum point discrepancy %.3g\n', ...
    strResults.ui32RandomizedCases, dMaxError);
end

function ExpectInvalidRayData_(strData)
%% SIGNATURE
% ExpectInvalidRayData_(strData)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Require an identified validation failure for a deliberately malformed payload.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% strData  Prepared geometry with a corrupted field.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% None; assert when the payload is accepted or fails outside the validator.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-10-2026  Pietro Califano, Codex (GPT-6)  Document malformed-payload acceptance checks.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ValidateTriangleRayData.
% -------------------------------------------------------------------------------------------------------------
arguments (Input)
    strData (1, 1) struct
end
try
    ValidateTriangleRayData(strData);
catch objError
    assert(startsWith(objError.identifier, 'ValidateTriangleRayData:'));
    return
end
error('testPreparedTriangleTracing:MissingError', 'Invalid numeric geometry was accepted.');
end
