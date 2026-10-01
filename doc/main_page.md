# SimulationGears_for_SpaceNav {#mainpage}

SimulationGears provides spacecraft navigation simulation models in MATLAB and
reusable native C++20 utilities, with optional CUDA, wrappers, and a ROS 2 Jazzy
overlay. The repository README is the entry point for installation and project
layout.

## Native build and consumption

```bash
cmake --preset native-cpu
cmake --build --preset native-cpu
ctest --preset native-cpu --output-on-failure --no-tests=error
```

Installed consumers use:

```cmake
find_package(SimulationGears_for_SpaceNav CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE
  SimulationGears_for_SpaceNav::SimulationGears_for_SpaceNav)
```

Use `CPU_ENABLE_NATIVE_TUNING=OFF` for portable CPU artifacts. CUDA is enabled
with `ENABLE_CUDA=ON`; OptiX remains an independent opt-in.

### Package metadata and container arguments

Python package metadata uses `PYTHON_PACKAGE_VERSION` from the existing CMake
version composer. Preserve prerelease qualifiers and build metadata in the
Python version instead of substituting only the numeric `PROJECT_VERSION`.

Container regeneration retains custom Docker `build.args` and unrelated build
fields. Update only `ROS_MODE`, `ROS_DISTRO` and `ROS_PROFILE` in the argument
mapping; remove those keys when ROS is disabled. Remove an empty argument mapping
without discarding any custom keys. Load JSON and JSONC through the shared helper
imported from `cpp_cuda_template_project`.

Run the focused checks against temporary inputs:

```bash
python3 -m pytest -q tests/python/test_devcontainer_build_args.py tests/python/test_python_package_version.py
```

Expected: eleven checks pass. These checks regenerate JSON and materialize
metadata with CMake; they do not build a container image or Python extension.

## Project contracts

- @ref md_doc_2logging documents CLogger.
- @ref md_doc_2testing__ci documents tests, CI, containers, ROS 2, and the
  downloadable documentation artifact.
- @ref md_doc_2ros2__overlay documents the optional ROS 2 overlay.
- @ref md_doc_2version__release documents version resolution and the canonical
  source TGZ release.

The `doc` preset generates HTML and XML locally. CI uploads those outputs as a
normal documentation artifact; it does not publish a website.

## MATLAB models and inputs

Use `matlab/SetupSimGears.m` to load the MATLAB model library and its MathCore
dependency. Keep host-side asset resolution and provenance separate from the
numeric runtime force inputs. Maintain interface contracts and examples here;
track review status and dated validation in the
[provider consolidation plan](developments/simulationgears_worktree_consolidation_plan_20260929.md).

### Kernel bundles and segmented SPK states

Prepare nominal trajectory inputs through the scenario manifest and generic
host-side utilities. Keep source names, external paths, hashes and acquisition
notes in the manifest; keep spacecraft SPK binaries outside the repository.
Use MICE and Java during host-side preparation. Load MathCore's
`ComputeFileSha256` adapter and bundled DataHash package through the existing
MathCore dependency. Keep file hashing in MathCore; SimulationGears owns bundle
selection and SPICE-pool validation. These utilities do not run in codegen
propagation.

#### Kernel selection and ownership

`LoadSelectedSpiceKernelBundle(scenario, bundleId)` resolves one bundle and its
trajectory through existing manifest/asset helpers. The default scenario
metakernel remains unchanged.

1. Verify SHA-256 for the metakernel, all declared ancillary files and the
   selected spacecraft trajectory before changing the SPICE pool.
2. Replace the process pool, load the ancillary metakernel from its directory
   and load the trajectory explicitly.
3. Check both the total count and the canonical loaded-path multiset against
   the verified files. Repeated entries remain significant.

Relative asset-root overrides resolve against the caller's MATLAB folder before
the loader changes it. Relative metakernel entries resolve against the
metakernel directory, independently of Java's process working folder.

Pre-load errors preserve the caller's pool. Loading or identity errors clear
partial loads; they do not restore the previous pool. Successful loading also
replaces that pool intentionally. Restore the caller's folder on every exit.
Returned provenance includes selected IDs, paths, hashes, trajectory metadata
and the verified loaded count.

#### Coverage, manoeuvres and boundary states

`ExtractSegmentedSpkManoeuvres` reads disjoint coverage intervals from the named
SPK and requires at least two arcs. Require a specific count with
`ui32ExpectedArcCount`; zero accepts any count of at least two.

- Reject gaps outside `(0, dMaximumGap]`, excessive position discontinuities
  and boundaries without a resolvable velocity jump.
- Check the active source handle at arc midpoints and both boundary endpoints.
  Check arc midpoint center, frame and segment type for consistency and against
  supplied expectations.
- Compute each impulse as post-endpoint velocity minus pre-endpoint velocity.
  Schedule it at the new arc's start. Return exactly `N - 1` impulses for `N` arcs.

Arcs are SPICE coverage intervals, not arbitrary SPK record boundaries. Touching
or overlapping records may merge into one interval. This interface does not
infer impulses inside continuously covered intervals or interpolate long gaps.
The caller owns loading and file-integrity verification; extraction and state
evaluation leave the pool unchanged.

`EvaluateSegmentedSpkState` preserves the requested timestamp separately from
the actual query endpoint. At an arc-start burn, select `charBoundarySide='pre'`
for the preceding arc's end, or `'post'` for the new arc's start. Ordinary queries
must lie within one supplied arc. Reject unloaded or higher-priority competing
spacecraft sources.

Use ET seconds for epochs and gaps, kilometres for positions and continuity
limits, and kilometres per second for velocities and impulses. Convert units at
the consumer boundary when needed.

#### Registered asset example

Run from this repository with MICE, the updated MathCore dependency and the
shared external asset root configured.
The manifest provides the source selection and NAIF metadata in this example.

```matlab
run('matlab/SetupSimGears.m');
strManifest = LoadScenarioDataManifest('Apophis');
strBundles = NormalizeManifestStructArray(strManifest.kernel_bundles);
strBundle = LoadSelectedSpiceKernelBundle('Apophis', char(strBundles(1).bundle_id));
strSource = strBundle.strTrajectoryMetadata;
strArcs = ExtractSegmentedSpkManoeuvres(strBundle.charTrajectoryFile, ...
    int32(strSource.spk_object_id), int32(strSource.spk_center_id), ...
    char(strSource.spk_frame), ...
    ui32ExpectedArcCount=uint32(strSource.coverage_arc_count), ...
    dExpectedCenterId=double(strSource.spk_center_id), ...
    charExpectedSegmentFrame=char(strSource.spk_frame));
[dState, strQuery] = EvaluateSegmentedSpkState(strBundle.charTrajectoryFile, ...
    int32(strSource.spk_object_id), int32(strSource.spk_center_id), ...
    char(strSource.spk_frame), strArcs.dBurnTimestamps(1), ...
    strArcs.dArcBounds, charBoundarySide='post');
fprintf('Kernels: %u, arcs: %u, impulses: %u, selected arc: %u\n', ...
    strBundle.ui32LoadedKernelCount, size(strArcs.dArcBounds, 2), ...
    size(strArcs.dBurnDeltaV, 2), strQuery.ui32ArcIndex);
cspice_kclear();
```

Expected output with the currently registered bundle:

```text
Kernels: 12, arcs: 4, impulses: 3, selected arc: 2
```

#### Focused validation

Run `testLoadSelectedSpiceKernelBundle` and
`testExtractSegmentedSpkManoeuvres` with real MICE. The extraction suite includes
synthetic type-9 fixtures and an optional installed-asset check controlled by
`RCS1_PHASE_D_KERNEL_ROOT`. Set that variable to the registered external kernel
directory to exercise the real trajectory. Report missing-asset assumptions as
skips, not successful validation.

Cover known SHA-256 vectors, nonempty caller-pool preservation, loaded-file
identity despite matching counts, relative roots/folder restoration, exact
four-arc/three-impulse shapes, gap/continuity/metadata/source failures, and
pre/post reference evaluation. These checks prepare trajectory inputs; they do
not run propagation, navigation or a physical-model qualification.

### Dataset and ephemeris rate transport

`SReferenceImagesDataset` accepts `dTargetAngVel_IN` and forwards it to the
mission-design base class. `FromSReferenceMissionDesign` retains those samples,
their timestamps and the source length-unit label; it does not rescale states
or infer rates from attitudes. Conversions from simulation-state data use this
same adapter and leave rates absent when their source supplies none.

`EphemeridesDataFactory` stores the following fields under
`strMainData.strAttData` for both the standard and RCS interpolation layouts:

| Field | Contract |
| --- | --- |
| `dAngVel_IN` | Finite 3-by-N source attitude-model rates in inertial coordinates, in rad/s |
| `dNominalAngVel_IN` | First source sample, retained for existing consumers |
| `dAngVelTimegrid` | N matching epochs in the actual interpolation domain: relative seconds, absolute seconds or absolute days |

Timegrid scaling does not rescale the rates. A source without rate data clears
these three fields from a reused dynamics structure, while retaining unrelated
attitude metadata. Keep the current model convention
`R_INfromTB(t) = Exp(-omega_IN*t) R_INfromTB(0)`; this transport fix does not
adopt the separate target-rotation extension.

For SPICE inputs, `DefineEnvironmentProperties` queries the inertial-to-target
state transform. `cspice_xf2rav` returns its rate in the input frame, already
inertial here, so apply only the existing model sign. See the
[NAIF frame contract](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/MATLAB/mice/cspice_xf2rav.html).

Run `testReferenceImagesDatasetRates` and `testEphemeridesDataFactory` after
`SetupSimGears`. Exercise the RCS branch with the real external
`EphCoeffsGeneration` and its `chebCoeffsGeneration` implementation. Use double polynomial
degrees until their separately recorded integer-type incompatibility is fixed.

### OBJ object selection

`charObjObjectNames` selects exact, case-sensitive OBJ `o` names. It is available through
`CShapeModel` (`file_obj` and OBJ `file_mesh`), `CShapeModel.LoadModelFromObj`, `LoadShapeMesh`,
`DefineShapeModel`, `CShapeModel.BuildSphericalHarmonicsGravityDataFromObj`,
`RunFitSpherHarmonicsToPolyhedronGravityFromObj` and `GenerateScenarioSHGravityCoefficients`.

Use `CShapeModel.BuildSphericalHarmonicsGravityDataFromObj` to load geometry and fit coefficients.
Use `RunFitSpherHarmonicsToPolyhedronGravityFromObj` to add holdout diagnostics, optional figures
and a printed summary. The standalone forwarding function
`FitSpherHarmonicsToPolyhedronGravityFromObj` has been removed; call the class builder directly
with the same arguments and selection options.

- Omit the option, or use `strings(1, 0)`, to retain existing full-mesh loading.
- `"surface"` selects only that object. An array selects the union in source face order, not
  caller-specified name order. Repeated declarations of the same name are combined.
- The scalar `""` selects unnamed faces, including faces preceding the first object record.
- Groups and material changes do not change object identity. Names with no face records,
  including partially missing requested unions, throw `SelectObjFaceRecords:MissingObjects`.
- Non-OBJ selection is rejected. No implicit fallback to the complete mesh occurs.

Selection precedes selected-face decoding, optional repair, simplification, bounds, volume
centering and gravity fitting. Referenced vertices are compacted in original vertex order;
face order, coordinates and winding are unchanged. Independent OBJ `vt`/`vn` arrays and selected
corner indices are retained by the auxiliary-aware reader. Selection does not generate normals,
sample maps, weld vertices, change units or recenter geometry.

`LoadShapeMesh` is geometry-only and repairs by default; request `bRepairMesh=false` for selection
without repair. The legacy `file_obj` path does not repair by default. Its existing supported
syntax remains positive-index triangles with consistent face-token layout and unindented records;
the general geometry reader supports relative indices and polygon triangulation. Selection does
not silently migrate callers between these contracts.

`LoadShapeMesh` accepts leading spaces and tabs for vertex and face records in
both full-mesh and selected-object loading. Check all face records before
accepting a fast result; route slash indices, relative indices and polygons
through the existing general reader. Repair may renumber vertices while
preserving ordered triangle coordinates and winding.

The shared selector scans object declarations and uses binary searches over sorted face offsets.
Scratch storage is one logical value per face plus object descriptors. Both readers reuse the
selector and vertex compactor; the legacy reader retains its bounded face decoder. The general
reader may use record-wise parsing when excluded faces contain polygons or slash indices.

#### Usage and units

Given a source OBJ whose coordinates are metres:

```matlab
run('matlab/SetupSimGears.m');
objSurface = CShapeModel("file_obj", "body.obj", "m", "km", true, ...
    "macro surface", true, charObjObjectNames="surface");
ui32Faces = objSurface.ui32triangVertexPtr.';
% Read kilometre coordinates; return volume in km3 and centroid in km.
dVertices = objSurface.dVerticesPos.';
[dVolume, dCentroid] = ComputeMeshModelVolumeAndCoM(ui32Faces, dVertices);
```

Expected: only referenced selected vertices/faces, in kilometre coordinates, with the source origin
preserved. With a geometry-only generic reader, use
`LoadShapeMesh('body.obj', bRepairMesh=false, charObjObjectNames="surface")`; its positions retain
the source units.

The scenario generator also forwards `charShapeAssetId`, `charAppearanceProfileId` and
`charAssetRootPath` to the registered asset resolver. Use actual object names from the chosen
OBJ; `uniform` selects appearance suitable for geometry-only preparation. Keep external payloads
outside the repository.

The scenario generator retains its existing volume-centering behavior and registered GM and
reference-radius normalization. Returned `strGeneration.strMesh` now records `charObjObjectNames`
and `charLengthUnits="km"` alongside the existing source metadata, original/centered volume
centroids, counts and radii. Legacy serialized field names are unchanged: the centroid/radius
fields use km, and the volume fields use km3. `DefineShapeModel` also records the requested names
in its metadata. Retain asset identity and transform provenance alongside this selection metadata.

When centering a rendering asset, apply the selected solid's centroid translation rigidly to
every object. When rendering at the source origin, retain the gravity-origin offset and apply it
when evaluating centroid-centered coefficients. Declare the target frame independently of OBJ.

### Panel Sun visibility

Use `ComputePanelSunVisibility` to estimate the illuminated fraction of each
triangle from equally weighted samples. Supply the spacecraft-to-Sun direction,
illuminated-side normals, sample positions and triangle vertices in one mesh
frame. Use one length unit for positions and the positive ray offset; the
spacecraft SRP preparation path uses metres. Face indices must agree across all
arrays. Back-facing and grazing faces return zero.

```matlab
dLowerTriangle = [0, 1, 0; 0, 0, 1; 0, 0, 0];
dFaceVertices = cat(3, dLowerTriangle, dLowerTriangle + [0; 0; 1]);
dSamplePoints = reshape(mean(dFaceVertices, 2), 3, 1, 2);
dNormals = repmat([0; 0; 1], 1, 2);
dVisibleFraction = ComputePanelSunVisibility([0; 0; 1], dNormals, ...
    dSamplePoints, dFaceVertices, 1e-8);  % Return [0; 1].
```

Treat occluding triangles as opaque on both sides, regardless of winding or
illumination. Exclude each emitting face. Shift the ray origin toward the Sun
by the supplied offset and count only intersections beyond that same tolerance
from the shifted origin. This excludes blockers within twice the offset of the
original sample. Use a point-source Sun; partial visibility is the fraction of
unblocked samples. Apply external eclipse factors separately. This geometry
utility leaves the existing propagation and panel-force models unchanged.

Run `testComputePanelSunVisibility` after `SetupSimGears` and adding
`tests/matlab/simulation_models` to the path. The harness checks analytic
occlusion, transforms, unit conversion, face ordering and invalid geometry.

### Spacecraft SRP response tables

Prepare geometry/optics with the owning panel builder, then generate a table on
the host. Keep file parsing, descriptor families and provenance outside runtime
force inputs. The body-frame spacecraft-to-Sun direction is the table query;
mass, pressure, attitude and eclipse remain separate physical inputs.

Resolve MathCore's `ComputeFileSha256` before generating host artifacts. The
generator records source-file and prepared-model identities; deploy the fixed
numeric payload for runtime evaluation.

Use `EvalRHS_SRPLutWithBias` for force and analytical partials in metre or
kilometre dynamics units. Supply resolved reference pressure, mass, nominal
acceleration bias, pointing data and transverse selection; keep filter state
indices and consider-mode flags in the filter adapter.
Select this model through the trailing `bUseSrpLut`, `strResponseLut`,
`strSrpData` and `bIncludeTransverse` inputs of `EvalRHS_InertialDynOrbit`.
Compose SRP independently of residual acceleration and report the selected
model through `strAccelInfo.dAccSRP` in the dynamics acceleration units.
Use the same field for cannonball and LUT calls, and for panel truth in
`EvalRHS_InertialDynMaxFidelity`. Keep one SRP acceleration in the diagnostic
record. Run `testOrbitalSrpLutModels` for composition, bias, unit, eclipse and
force-reporting checks without a filter-library dependency.

```matlab
strLut = BuildSrpResponseLut(strPanel, dReferenceArea, 5, bIncludeTransverse=true);
strPayload = strLut.strResponseLut;
[dScalarForce, dCr] = EvaluateSrpResponseLut([1; 0.2; 0.3], strPayload);
[dFullForce, dSameCr] = EvaluateSrpResponseLut([1; 0.2; 0.3], strPayload, true);
[dJac, ~, ~, dForce] = EvalJac_SrpResponseLut([1; 0.2; 0.3], strPayload, true);
```

Return force divided by pressure in square metres, a dimensionless scalar
coefficient and a 3-by-3 response/query partial. Preserve the scalar law and
remove each node's parallel force before interpolating vector samples:

```text
s_i = node spacecraft-to-Sun unit direction in body coordinates
h_i = direct panel force divided by pressure
t_i = (I - s_i s_i') h_i
s = normalized query spacecraft-to-Sun direction
f_scalar = -A_ref C(s) s
f_transverse = (I - s s') Interp(t_i)
f_full = f_scalar + f_transverse
```

Store `t_i` as `dTransverseForcePerPressure`. Interpolate it and `C` with the
same four bilinear weights; differentiate normalization, interpolation and
projection analytically. Keep full nodal forces as `dForcePerPressure` only
in the host artifact for independent diagnostics. Purely radial models then
have zero transverse response even between nodes.

#### Fixed storage and code generation

Pack six numeric fields: active azimuth/elevation counts, their axes,
`dEffectiveCr` and `dReferenceArea_m2`. Add `dTransverseForcePerPressure` only
when `bIncludeTransverse=true`. Default both host generation and packing to
scalar storage. Preserve finite values, uniform axes, exact seam/pole equality,
zero padding and nodal transverse orthogonality through ordinary validation.

Default packing capacity is 361 by 181 nodes. The host generator instead
matches storage to grid dimensions by default. A five-degree grid uses 73 by
37 nodes: 22,504 numeric bytes for scalar storage or 87,328 with transverse
samples. At maximum capacity these sizes are 527,080 and 2,095,264 bytes,
respectively, before any platform-specific struct padding. Validate prepared
data once, outside repeated force calls.

Use `SSrpResponseLut` for the selected fixed layout in each generated target.
Keep scalar and transverse artifacts separate; use identical capacities and
field order across entry points within a build. Name resolved physical inputs
`SSrpData` and pointing data `SSrpPointing`. Pass transverse selection separately as a constant input
through `EvalRHS_SRPLutWithBias` and `EvalRHS_InertialDynOrbit`; keep pressure,
mass, bias, pointing and states as numerical inputs.

```matlab
CodegenSrpResponseLut(charScalarRoot, strPayload);
CodegenSrpResponseLut(charTransverseRoot, strPayload, bIncludeTransverse=true);
CodegenSrpResponseLut(charJacRoot, strPayload, bIncludeTransverse=true, ...
    charEntryPoint='EvalJac_SrpResponseLut');
CodegenSrpResponseLut(charLibraryRoot, strPayload, charTarget='lib', ...
    bIncludeTransverse=true, bFreezeTable=false);
```

Freeze inclusion during code generation independently of table embedding.
MEX interfaces omit the inclusion flag and, for embedded builds, the table.
Runtime-table builds accept numerical values with the selected fixed layout;
scalar targets contain neither the transverse array nor its computations.
Use `GenerateSrpLutDirections` for deterministic independent sphere queries.

MEX builds default to one output; select another leading output count with
`ui8OutputCount`. C++ libraries retain all outputs by default. Fix the output
prefix at generation time: requesting fewer outputs from an existing MEX does
not specialize its compiled calculations. Disable dynamic allocation and
variable sizing in numerical targets; MEX gateways allocate MATLAB outputs.

#### Physical acceleration and derivative conventions

`ComputeSrpLutAcceleration` evaluates SI acceleration and analytical position,
attitude and mass partials. Supply spacecraft-to-Sun displacement in metres,
body-to-inertial rotation, mass in kg, and current pressure in N/m². Select the
inverse-square pressure partial explicitly. Supply `dR/dr` when attitude depends
on position; otherwise attitude is fixed. Gate target eclipse through the
calling dynamics model's `bIsInEclipse`. Retain spacecraft self-shadowing in the
prepared table; no partial-eclipse fraction or gradient is required.
Right body rotation error means `R(delta)=R exp(skew(delta))`.
Request only the needed leading outputs: acceleration, position partial,
attitude partial, mass partial, then regularity. Compile-time `nargout` removes
unrequested derivative branches. Force-only calls skip all Jacobian work.

At interpolation knots/seam, use the selected one-sided cell derivative and
return false regularity. Near a pole, tilt only the lookup direction by 1e-8 rad
on a fixed golden-angle meridian whenever normalized XY radius is below 2.5e-9.
Use no random generator or mutable state. Preserve the physical Sun direction
in scalar force and transverse projection, and include the constant lookup
rotation in the derivative chain. Return false regularity to identify adjusted
queries. This defines an artificial local branch with a small switch boundary;
it does not establish a unique derivative of the original angular interpolant.
The filter uses the adjusted force/Jacobian consistently without pole rejection.

Run focused owner checks through `SetupSimGears` and the test directory
`tests/matlab/simulation_models/accelerations/srp_lut`. Independent constant-law
checks, regular-point finite differences, boundary semantics, compact/legacy
payload parity and source/generated parity are separate validation gates.

### MATLAB dynamics and Jacobian names

Use `EvalRHS_…` for right-hand-side kernels and `EvalJac_…` for Jacobians.
Match each primary function name to its source filename. Update callers,
function handles and generated entry points with the provider; rebuild MEX
artifacts compiled from these entry points. The previous case spellings have
no forwarding aliases.
