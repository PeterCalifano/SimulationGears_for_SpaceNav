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
