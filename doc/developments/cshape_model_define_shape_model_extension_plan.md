# `CShapeModel` / `DefineShapeModel` Extension Plan

## Current Position

- [x] `CShapeModel` already loads meshes from `.obj`, SPICE, `struct`, and `.mat`, and now supports optional load-time mesh simplification.
- [x] `CShapeModel.BuildSphericalHarmonicsGravityData(...)` and `FitSpherHarmCoeffToPolyhedrGrav(...)` already compute mesh volume, derive `density` from `mu` or `mu` from `density`, and reject inconsistent physical inputs.
- [x] `DefineShapeModel(...)` already routes known scenarios and fills mesh-path, reference-size, and ellipsoid-shape defaults.
- [x] `CScenarioGenerator.LoadDefaultScenarioData(...)` already owns known-scenario gravity constants (`dGM`, `dRefRadius`) and a partial SH-coefficient path.
- [x] `CShapeModel.BuildAndSetSphericalHarmonicsGravityData(...)` now provides an instance-level build-and-cache path with degree 4 as the default.
- [x] `DefineShapeModel(...)` now initializes SH gravity data by default for loaded shapes when known-scenario or explicit physical inputs are available.
- [x] Known-scenario `dGravParam` lookup is isolated in `GetShapeModelScenarioGravityDefaults(...)` so other utilities can share the registry instead of duplicating constants.
- [x] `DefineShapeModel(...)` now resolves scenario metadata through `CScenarioRegistry`, while its legacy branch body still preserves existing mesh-loading behavior.
- [x] `DefineShapeModel(...)` exposes a first-pass `FromShape` custom-shape contract for one arbitrary user-provided OBJ mesh.
- [x] Known-scenario SH support now has registry-backed payloads and degree metadata for available public models.
- [x] `GenerateScenarioRegistrySHliterals(...)` prints reusable fixed-scenario gravity / SH payloads from archived public source files.

## Where We Were Headed

- [ ] Preserve all current known-scenario behavior and call sites unless a compatibility alias is required.
- [x] Add one explicit `FromShape` custom-shape path that accepts exactly one shape source.
- [x] Treat `EnumScenarioName.NotDefined` as passthrough/backward compatibility, not as the primary custom-shape API.
- [ ] Keep `DefineShapeModel(...)` thin by reusing existing mesh, gravity, and SH utilities instead of reimplementing them there.
- [ ] Separate runtime loading from offline generation of fixed-scenario hardcoded gravity / SH data.
- [ ] End with one authoritative scenario-data seam reused by both shape loading and environment/gravity setup.

## Stage Ledger

### Stage 0: Ownership And Baseline Audit

- [ ] Enumerate every field currently assigned inside `DefineShapeModel(...)` and classify it as mesh-routing data, geometric metadata, physical metadata, SH metadata, or Blender-only data.
- [ ] Compare `DefineShapeModel(...)` known-scenario physical constants against `CScenarioGenerator.LoadDefaultScenarioData(...)` and record mismatches.
- [ ] Resolve naming / compatibility drift for supported bodies, including `Didymos` vs `Dydimos` and any `Bennu` / `Bennu_OREx` mismatch at the loader boundary.
- [ ] Capture a regression baseline for current known-scenario outputs before changing ownership.
- [ ] Gate: add or update tests that assert current `DefineShapeModel(...)` output shape for supported fixed scenarios before refactor begins.

### Stage 1: Shared Scenario Registry

- [x] Introduce `CScenarioRegistry` as the static MATLAB scenario/custom-spec resolver used by `DefineShapeModel(...)`, `CScenarioGenerator.LoadDefaultScenarioData(...)`, and `CShapeModel` SH initialization.
- [x] Registry specs include canonical enum, aliases, SPICE target name, fixed frame, GM, reference radius, ellipsoid axes or shape matrix, mesh source rules, Blender path rules, and SH metadata.
- [x] Move known-scenario physical metadata ownership out of the large per-scenario branch body and into the registry.
- [x] Store SH availability with both `ui32SourceMaxDegree` and `ui32HardcodedMaxDegree`; checked-in SH payloads are capped at degree/order 16.
- [x] Preserve existing first and second outputs from `DefineShapeModel(...)`.
- [ ] Keep Blender-path handling separate from mesh / physics metadata so custom-shape support does not inherit unnecessary Blender requirements.
- [ ] Gate: known-scenario tests still return the same mesh path, reference size, target shape matrix, and output-unit behavior as before.

### Stage 2: `FromShape` Custom Shape Contract

- [x] Add `EnumScenarioName.FromShape` as the explicit custom-shape path without weakening current known-scenario routing.
- [x] Keep `EnumScenarioName.NotDefined` as a passthrough/legacy no-defaults path.
- [x] Assert that exactly one mesh source is provided for the custom path.
- [ ] Accept custom physical inputs in a single consistent contract:
- [ ] `dMass_kg`
- [ ] `dDensity_kgm3`
- [ ] `dVolume_m3` as optional override
- [ ] `dGravParam_m3mps2`
- [ ] If `dVolume_m3` is omitted, compute enclosed volume from the loaded mesh.
- [ ] If multiple physical inputs are provided, validate consistency instead of silently selecting one.
- [ ] Reject open / invalid meshes when volume-dependent derivation is requested and the mesh cannot support it.
- [ ] Gate: add focused tests for one-source assertion, missing-input rejection, consistent derivation, and inconsistent-input rejection.

### Stage 3: Gravity-Enriched Shape Metadata

- [ ] Add one optional metadata output / cache path that can expose:
- [ ] mesh source and units
- [ ] reference radius
- [ ] enclosed volume
- [ ] mass
- [ ] density
- [ ] gravitational parameter
- [ ] optional SH payload
- [ ] Prefer returning a metadata struct (or one optional extra output) over bloating `CShapeModel` with many new scalar fields unless downstream usage proves repeated object-side storage is necessary.
- [ ] Reuse `ComputeMeshModelVolumeAndCoM(...)`, `CShapeModel.BuildPolyhedronGravityData(...)`, and existing SH utilities instead of adding parallel implementations.
- [ ] Gate: add tests that custom metadata is computed correctly and known scenarios still load without requiring enrichment.

### Stage 4: SH Runtime Integration

- [x] Add a default runtime compute path for shape loading through `DefineShapeModel(...)` with `bInitSphericalHarmonicsGravityData=true` and degree 4.
- [x] For mesh-backed shapes, compute and cache SH data only by delegating to `CShapeModel.BuildAndSetSphericalHarmonicsGravityData(...)`.
- [x] Generalize the runtime SH mode beyond the current boolean into explicit values: `none`, `registry`, `compute`, and `auto`.
- [x] In `auto`, prefer checked-in registry SH data when `requestedDegree <= ui32HardcodedMaxDegree`; otherwise recompute from the loaded shape model.
- [x] Store checked-in SH coefficients in repo-native unnormalized `[Clm, Slm]` column-pair format with enough numeric digits to avoid precision loss in propagation tests.
- [ ] Revisit default-on known-scenario SH computation once hardcoded payloads exist, because runtime fitting can be expensive for full meshes.
- [x] Replace the incomplete `CScenarioGenerator.LoadSpherHarmCoefficients(...)` runtime path with registry lookup inside `LoadDefaultScenarioData(...)`.
- [x] Gate: add tests for custom SH build/cache, known-scenario hardcoded SH retrieval, and unavailable higher-degree registry fallback behavior.

### Stage 4b: Scenario-Driven Dataset Builder

- [x] Add one public static `CScenarioGenerator` entry point that takes scenario enum/name, spacecraft state, timestamps, optional reference data, and `bCompleteFromReferences=false`.
- [x] Return `SReferenceImagesDataset` plus environment/dynamics structs and registry metadata.
- [x] Keep reference completion explicit: real provided data wins, and missing fields are filled from simple registry/default references only when requested.
- [x] Gate: add smoke test with reference completion enabled.

### Stage 5: One Offline Generation Script

- [x] Add one script only that loops over supported public SH source files and prints reusable MATLAB SH literals.
- [ ] Extend the script to print reference radius, enclosed volume, and mass / density / `GravParam` consistency data from active meshes.
- [ ] The script must use the same loader path that runtime code uses when mesh-derived metadata is added.
- [x] The script output is deterministic and paste-ready for the checked-in registry helper.
- [x] Gate: add/run a smoke check for a lightweight subset and verify the printed format is stable.

### Stage 6: Regression Closure And Documentation

- [ ] Update loader docstrings and examples for known-scenario and custom-shape usage.
- [ ] Document the exact precedence rules for custom physical inputs and unit expectations.
- [ ] Document the SH runtime modes and the difference between hardcoded and computed SH payloads.
- [ ] Verify that existing callers using only known scenarios do not require changes.
- [ ] Gate: rerun focused no-regression suites before closing the work.

## Automated Gate Inventory

- [x] Keep and rerun `tests/matlab/simulation_models/testCShapeModelSimplifyMesh.m`.
- [x] Keep and rerun `tests/matlab/simulation_models/accelerations/testFitSpherHarmCoeffToPolyhedrGrav.m`.
- [x] Add focused coverage for the `CShapeModel` instance SH build/cache method and the default `DefineShapeModel(...)` SH initialization path.
- [ ] Add one new focused test file for `DefineShapeModel(...)` custom-shape and metadata behavior.
- [x] Add one new focused test file or section for known-scenario preservation after the refactor.
- [ ] Add a smoke test for the single offline generation script.

## Manual And Environment-Backed Coverage

- [ ] Validate at least one SPICE-backed known scenario on a machine with the expected kernels available.
- [ ] Validate at least one OBJ-backed known scenario with no SPICE dependency.
- [ ] Validate one custom closed triangular mesh with mass-only input.
- [ ] Validate one custom closed triangular mesh with density-only input.
- [ ] Validate one custom mesh with SH mode set to `compute`.
- [ ] Confirm that Blender-path outputs remain unchanged for fixed scenarios that currently use them.

## Candidate Additions For Review

- [x] Custom path decision: use `EnumScenarioName.FromShape`; keep `EnumScenarioName.NotDefined` as passthrough/backward compatibility.
- [ ] Decide whether enriched physical metadata should be a third output from `DefineShapeModel(...)` or a dedicated helper that wraps it.
- [x] Checked-in fixed-scenario SH payloads live in the scenario registry, not beside `CScenarioGenerator`.
- [x] Maximum checked-in SH degree is degree/order 16, with each body recording its actual `ui32HardcodedMaxDegree`.
- [ ] Decide whether the loader should auto-center custom meshes for SH fitting or require pre-centered input and fail fast otherwise.

## Definition Of Done

- [x] Known-scenario calls to `DefineShapeModel(...)` still work without behavior regressions in focused tests.
- [ ] One custom-shape path exists and supports one-mesh plus physical-input derivation cleanly.
- [x] Physical metadata and SH handling reuse existing gravity / mesh utilities instead of duplicating them.
- [x] One offline script generates and prints reusable fixed-scenario hardcoded SH data.
- [x] Tests cover both preserved behavior and the first new custom-shape workflow.
- [x] Affected external consumers are listed with proposed follow-up updates for EstimationGears, nav-backend, cosmica-simulator, and nav-frontend.
