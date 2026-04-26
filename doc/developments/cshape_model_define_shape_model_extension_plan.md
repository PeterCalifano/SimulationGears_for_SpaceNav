# `CShapeModel` / `DefineShapeModel` Extension Plan

## Current Position

- [x] `CShapeModel` already loads meshes from `.obj`, SPICE, `struct`, and `.mat`, and now supports optional load-time mesh simplification.
- [x] `CShapeModel.BuildSphericalHarmonicsGravityData(...)` and `FitSpherHarmCoeffToPolyhedrGrav(...)` already compute mesh volume, derive `density` from `mu` or `mu` from `density`, and reject inconsistent physical inputs.
- [x] `DefineShapeModel(...)` already routes known scenarios and fills mesh-path, reference-size, and ellipsoid-shape defaults.
- [x] `CScenarioGenerator.LoadDefaultScenarioData(...)` already owns known-scenario gravity constants (`dGM`, `dRefRadius`) and a partial SH-coefficient path.
- [ ] `DefineShapeModel(...)` still duplicates known-scenario physical metadata instead of reusing one authoritative scenario-data source.
- [ ] `DefineShapeModel(...)` does not yet expose a clean custom-shape contract for one arbitrary user-provided mesh plus physical inputs.
- [ ] Known-scenario SH support is incomplete and split between hardcoded placeholders and partially implemented loader logic.
- [ ] There is no single offline script that computes and prints reusable fixed-scenario gravity / SH payloads for later hardcoding.

## Where We Were Headed

- [ ] Preserve all current known-scenario behavior and call sites unless a compatibility alias is required.
- [ ] Add one explicit custom-shape path that accepts exactly one shape source and derives the remaining physical data consistently.
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

### Stage 1: Shared Scenario Spec Layer

- [ ] Introduce one small internal scenario/custom-spec resolver used by `DefineShapeModel(...)`.
- [ ] Move known-scenario physical metadata ownership out of the large per-scenario branch body and into the shared resolver or existing scenario generator seam.
- [ ] Preserve existing first and second outputs from `DefineShapeModel(...)`.
- [ ] Keep Blender-path handling separate from mesh / physics metadata so custom-shape support does not inherit unnecessary Blender requirements.
- [ ] Gate: known-scenario tests still return the same mesh path, reference size, target shape matrix, and output-unit behavior as before.

### Stage 2: Custom Shape Contract

- [ ] Add one explicit custom-shape path, likely under `EnumScenarioName.NotDefined` or a compatibility alias, without weakening current known-scenario routing.
- [ ] Assert that exactly one mesh source is provided for the custom path.
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

- [ ] Add a runtime SH mode for shape loading with explicit values such as `none`, `hardcoded`, and `compute`.
- [ ] For custom shapes, allow SH computation only by delegating to existing `CShapeModel.BuildSphericalHarmonicsGravityData(...)`.
- [ ] For fixed known scenarios, prefer checked-in hardcoded SH data when available.
- [ ] Keep expensive SH computation opt-in for known scenarios unless explicitly requested.
- [ ] Refactor or replace the incomplete `CScenarioGenerator.LoadSpherHarmCoefficients(...)` path so the runtime SH contract is complete and does not rely on placeholders.
- [ ] Gate: add tests for custom SH build/cache, known-scenario hardcoded SH retrieval, and empty-SH behavior when no hardcoded payload exists.

### Stage 5: One Offline Generation Script

- [ ] Add one script only that loops over all supported fixed scenarios and prints reusable MATLAB literals / structs for:
- [ ] reference radius
- [ ] enclosed volume
- [ ] mass / density / `GravParam` consistency data
- [ ] optional SH payloads and degree metadata
- [ ] The script must be offline and separate from runtime `DefineShapeModel(...)`.
- [ ] The script must use the same loader path that runtime code uses, so generated hardcoded data matches active meshes.
- [ ] The script output should be deterministic and paste-ready for one checked-in data file or helper.
- [ ] Gate: add a smoke test or deterministic-output check for a lightweight subset and verify the printed format is stable.

### Stage 6: Regression Closure And Documentation

- [ ] Update loader docstrings and examples for known-scenario and custom-shape usage.
- [ ] Document the exact precedence rules for custom physical inputs and unit expectations.
- [ ] Document the SH runtime modes and the difference between hardcoded and computed SH payloads.
- [ ] Verify that existing callers using only known scenarios do not require changes.
- [ ] Gate: rerun focused no-regression suites before closing the work.

## Automated Gate Inventory

- [ ] Keep and rerun `tests/matlab/simulation_models/testCShapeModelSimplifyMesh.m`.
- [ ] Keep and rerun `tests/matlab/simulation_models/accelerations/testFitSpherHarmCoeffToPolyhedrGrav.m`.
- [ ] Add one new focused test file for `DefineShapeModel(...)` custom-shape and metadata behavior.
- [ ] Add one new focused test file or section for known-scenario preservation after the refactor.
- [ ] Add a smoke test for the single offline generation script.

## Manual And Environment-Backed Coverage

- [ ] Validate at least one SPICE-backed known scenario on a machine with the expected kernels available.
- [ ] Validate at least one OBJ-backed known scenario with no SPICE dependency.
- [ ] Validate one custom closed triangular mesh with mass-only input.
- [ ] Validate one custom closed triangular mesh with density-only input.
- [ ] Validate one custom mesh with SH mode set to `compute`.
- [ ] Confirm that Blender-path outputs remain unchanged for fixed scenarios that currently use them.

## Candidate Additions For Review

- [ ] Decide whether the custom path remains under `EnumScenarioName.NotDefined` or whether a dedicated `Custom` / `Arbitrary` enum alias is preferable.
- [ ] Decide whether enriched physical metadata should be a third output from `DefineShapeModel(...)` or a dedicated helper that wraps it.
- [ ] Decide whether checked-in fixed-scenario SH payloads should live beside `CScenarioGenerator` data or in a dedicated shape-scenario data file.
- [ ] Decide the maximum checked-in SH degree per fixed scenario so runtime payload size stays controlled.
- [ ] Decide whether the loader should auto-center custom meshes for SH fitting or require pre-centered input and fail fast otherwise.

## Definition Of Done

- [ ] Known-scenario calls to `DefineShapeModel(...)` still work without behavior regressions.
- [ ] One custom-shape path exists and supports one-mesh plus physical-input derivation cleanly.
- [ ] Physical metadata and SH handling reuse existing gravity / mesh utilities instead of duplicating them.
- [ ] One offline script generates and prints reusable fixed-scenario hardcoded data.
- [ ] Tests cover both preserved behavior and the new custom-shape workflow.
