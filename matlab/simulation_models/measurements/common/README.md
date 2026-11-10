# Prepared triangle tracing

Prepare geometry once with `BuildTriangleRayData`, validate it with
`ValidateTriangleRayData`, and reuse `TraceTriangleRay` for opaque any-hit or
nearest-hit queries. Both traversal modes use one Moller–Trumbore kernel.
Preserve original triangle IDs; nearest ties select the lowest source ID.
A miss returns `false`, range `-1`, a zero point and ID zero.

The BVH builder and nearest-hit traversal adapt RCS-1 commit
`9b6a2d47c572396982b5fa3249972c34040ea852`, specifically
`functions/LidarRangefinder/BuildLidarBvh.m` and `RayTraceTriangMesh.m`.
SimulationGears owns the generic implementation: geometry and query values remain
runtime data, the traversal stack scales with tree depth, and sensor policy stays
in the LiDAR model. The donor checkout is unchanged.

```matlab
dTriangle = [0,1,0; 0,0,1; 0,0,0];
strData = BuildTriangleRayData(dTriangle, true);
strQuery = struct('dDirection',[0;0;-1], 'bAnyHit',false, ...
    'bTwoSided',true, 'dMinDistance',0, 'dMaxDistance',Inf, ...
    'ui32IgnoreTriangle',uint32(0));
[bHit,dRange] = TraceTriangleRay(strData,[0.2;0.2;1],strQuery);
% Output: bHit = true, dRange = 1.
```

## Numeric contract

The payload has exactly 13 fields: active triangle/node counts, vertex zero,
two edges, node minimum/maximum bounds, left/right children, leaf start/count,
source permutation and a logical BVH selector. Arrays are fixed at preparation;
values and active counts remain runtime data. The host builder compacts unused
node capacity. The balanced median tree has eight-triangle leaves and traversal
uses a bounded stack. Validate arrays/topology before compiled queries.
The builder partitions source IDs in place; it does not sort each complete subtree.
Traversal reuses direction reciprocals and conservative origin-dependent bounds.
Padding affects candidate selection; exact triangle and interval tests decide hits.

Use one length unit and transform the ray into the stored mesh frame. A unit
ray direction returns range in that length unit; otherwise the result is the
ray parameter. The interval is open below and closed above. Keep the existing
absolute determinant tolerance (`eps`, or `2*eps` for shadow rays); this is not
an adaptive precision or watertight intersection algorithm. Extreme geometric
scales may require a separately qualified tolerance policy.

Rebuild after vertex, topology or unit changes. Rigid pose changes reuse the
payload. `CShapeModel.prepareRayTracingData` caches it; existing geometry
mutators clear the cache and `getRayTracingModel` rejects changed unit metadata.
The cache is private and transient. MAT saves omit it, and `CBaseDatastruct`
excludes transient properties from struct, JSON and YAML exports. Loading a saved
shape restores its geometry; prepare its tracing cache again before compiled use.

## Code generation and LiDAR

`CodegenTriangleTracing` builds `TraceTriangleRay` or, with `bLidar=true`, the
complete `LaserRangefinderModel`. It does not capture constant mesh values.
MEX builds use direct numeric inputs and generate a MATLAB facade with the
existing public interface. This removes struct-field marshalling, although the
generated argument-protection code still copies shared arrays. A `TODO (PC)`
marks that remaining cost. The traversal stack is fixed at 34 uint32 entries.
Native-library builds retain the prepared struct interface and fixed arrays.
Both builds disable numeric heap allocation and variable sizing. Load MathCore's
`ComputeFileSha256` provider, then select an empty output directory. Sensor
builds save a source/capacity manifest next to `LaserRangefinderPrepared_MEX`.
Consumers must check that manifest before selecting a generated binary.
Nav-Backend's adapter checks the source hashes, fixed capacities and compiled
kernel selected by the facade. Source evaluation remains usable when no prepared
MEX is selected. The measured speed improvement uses the compiled sensor.

Build the prepared sensor from the scenario's loaded shape before deployment:

```matlab
objShape = objShape.prepareRayTracingData(true);
strModel = objShape.getRayTracingModel();
charMatlabRoot = fileparts(which('SetupSimGears'));
charBuildRoot = fullfile(charMatlabRoot, 'mex', 'lidar-prepared-bennu');
strBuild = CodegenTriangleTracing(charBuildRoot, strModel.strRayData, bLidar=true);
addpath(strBuild.charOutputRoot);
% LaserRangefinderPrepared_MEX now resolves to the generated sensor facade.
```

The destination must be empty and the configured MathCore source provider must
already be on the path. Keep the facade, numeric MEX and signature together.
Use a new empty build directory after source or capacity changes, then select
that directory before the run. Keep loaded binaries and source paths stable
during the run. Generated artifacts stay under `matlab/mex/` and outside commits.

The public mesh tracer now returns the nearest positive hit independently of
record order and the nominal mesh centre. Sensor noise, bias and validity bounds
remain in `LaserRangefinderModel`; missing geometry is always invalid. The
previous raw-mesh MEX is not a substitute for the prepared signature or the
corrected nearest-hit contract.

## Selection and qualification

Use a flat scan for the 68-face spacecraft. Its Sun-parallel shadow evaluator
caches direction coefficients and conservatively rejects blockers outside each
emitter's projected sample bundle. Keep every potential blocker two-sided.
This changes neither quadrature nor ray intersection semantics.

Qualify each asset using the same triangles and rays for flat and BVH traversal.
Compare complete LiDAR calls, including MEX argument marshalling. Require matching
hit flags, ranges and points, then select the faster mode. The user accepted
the measured approximately 9% complete-call improvement for the campaign's
17,866,836-face Bennu mesh. Report preparation cost and peak memory separately.
The final comparison used 16 rays from 1 to 35 km and five warm timing repeats:
flat median 0.603631 s/call, BVH median 0.547505 s/call, a 9.298% reduction.
Preparation took 141.709 s for 6,373,543 nodes. The complete qualification process
peaked at 24.21 GiB resident memory, including loaded geometry, both traversal
payloads and code generation. Synthetic checks covered 3,072 flat/BVH mode/unit
cases, 512 RCS-1 source comparisons and 2,048 mutable generic MEX queries.
All compared hit flags, ranges and points matched. The original process exited
at the earlier 10% assertion; the user accepted this measured improvement.
Earlier measurements belong to the preceding implementation; remeasure after
changes to preparation, traversal or code generation. LiDAR results do not qualify
spacecraft shadowing or a flight processor.
