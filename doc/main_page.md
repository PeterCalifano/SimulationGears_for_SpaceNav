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
