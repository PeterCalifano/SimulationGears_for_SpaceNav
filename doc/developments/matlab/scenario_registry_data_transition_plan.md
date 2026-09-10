# Scenario Registry Data Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. User constraint: do not commit; review the commit split at the end.

**Goal:** Move scenario-data ownership into SimulationGears, make scenario identities explicit, and add runnable shape-backed scenario metadata for Apophis/Arrokoth/67P/Toutatis/Eros without creating a broad ingestion framework.

**Architecture:** Keep `CScenarioRegistry` as the public scenario metadata source and add a small manifest/data-root layer beside it. Large/public data files live under SimGears `data/` as ignored fetched assets; tracked files are manifests, fetch metadata, tests, and MATLAB utilities.

**Tech Stack:** MATLAB R2023b, `matlab.unittest`, JSON manifests via `jsondecode`, existing `CBaseDatastruct` serialization conventions, existing `CShapeModel`/`DefineShapeModel` loaders.

## Global Constraints

- Do not commit.
- Preserve unrelated current open changes and do not revert user work.
- Use checkbox syntax in this plan.
- Treat `ApophisElongated` as first-class; remove `ItokawaModified`.
- Remove `bLoadModifiedVariant` and update consumers.
- Shape defaults prefer maximum practical public fidelity with a preferred `< 1 GB` artifact/archive size.
- Do not commit large raw archives or fetched science assets; use manifests and ignored `data/` asset folders.
- Transition relevant nav-backend scenario data to SimGears ownership instead of keeping a nav-backend fallback.
- Keep current SH file-backed generator work as its own commit candidate.

---

## Current Open Changes Included

- [ ] **Already staged SH candidate:** `SSphericalHarmonicsGravityData`, `CScenarioGenerator`, and `testSSphericalHarmonicsGravityData` are staged as the current file-backed SH/generator batch. This plan will update that staged candidate for enum scenario and length-unit cleanup, then it should be restaged as one clean commit candidate.
- [ ] **Scenario/shape-model open work:** `CShapeModel`, `DefineShapeModel`, `testCShapeModelSimplifyMesh`, `FitSpherHarmonicsToPolyhedronGravityFromObj`, and `RunFitSpherHarmonicsToPolyhedronGravityFromObj` have unstaged changes related to physical metadata, SH fitting, and shape-model setup. This plan will touch `CShapeModel`, `DefineShapeModel`, and scenario tests; the fitter/run script changes should remain a separate review group unless directly needed.
- [ ] **Dynamics/noise open work:** inertial max-fidelity dynamics files, SRP tests, finite-burn/Gauss-Markov files, dynamics tests, photometry tests, propagator tests, and the MathCore submodule pointer are present in the dirty tree. This plan must not fold them into the scenario/data commits.
- [ ] **New tools folder:** `tools/` is currently untracked. This plan may add scenario-data tools there; final review must distinguish new scenario-data tools from pre-existing untracked tool files.

## Task 1: Plan File And Baseline Tests

**Files:**
- Create: `doc/developments/scenario_registry_data_transition_plan.md`
- Read: `matlab/simulation_management/CScenarioRegistry.m`
- Read: `matlab/simulation_models/shape_models/builders/DefineShapeModel.m`
- Read: `matlab/simulation_models/accelerations/SSphericalHarmonicsGravityData.m`
- Test: focused tests listed below

**Interfaces:**
- Produces this plan as the controlling implementation record.
- Captures dirty-tree commit boundaries so implementation does not mix unrelated work.

- [ ] Run baseline focused tests before behavior edits:
  - `runtests('tests/matlab/simulation_management/testCScenarioRegistry')`
  - `runtests('tests/matlab/simulation_models/accelerations/testSSphericalHarmonicsGravityData')`
  - `runtests('tests/matlab/simulation_models/testCShapeModelSimplifyMesh')`

## Task 2: Shared Length Units And SH Data Enum Cleanup

**Files:**
- Create: `matlab/general_utils/datastructs/EnumLengthUnits.m`
- Create/modify: `tests/matlab/general_utils/datastructs/testEnumLengthUnits.m`
- Modify: `matlab/simulation_models/accelerations/SSphericalHarmonicsGravityData.m`
- Modify: `tests/matlab/simulation_models/accelerations/testSSphericalHarmonicsGravityData.m`
- Modify: length-unit arguments touched in `CScenarioRegistry`, `CShapeModel`, and `DefineShapeModel`

**Interfaces:**
- `EnumLengthUnits` enumeration entries: `km`, `m`.
- Public APIs accept `EnumLengthUnits`, `string`, or `char`.
- Saved SH schema remains text-compatible with `charScenarioName` and `charLengthUnits`.
- Runtime SH data stores/normalizes scenario as `EnumScenarioName` and length unit as `EnumLengthUnits`.

- [ ] Add failing tests for enum/string/char unit normalization and SH saved-struct compatibility.
- [ ] Add `EnumLengthUnits` beside `EnumFrameName`.
- [ ] Update touched APIs to normalize length units through the enum while preserving text in public metadata.
- [ ] Update `SSphericalHarmonicsGravityData` validation and serialization to use enum scenario/unit internally.
- [ ] Run focused SH and enum tests.

## Task 3: First-Class Scenarios And Legacy Alternate Removal

**Files:**
- Modify: `matlab/simulation_management/EnumScenarioName.m`
- Modify: `matlab/simulation_management/CScenarioRegistry.m`
- Modify: `matlab/simulation_models/shape_models/builders/DefineShapeModel.m`
- Modify: nav-backend consumer wrappers if touched in this workspace review only; do not patch nav-backend unless explicitly included later.
- Modify: `tests/matlab/simulation_management/testCScenarioRegistry.m`
- Modify: `tests/matlab/simulation_models/testCShapeModelSimplifyMesh.m`

**Interfaces:**
- Remove `EnumScenarioName.ItokawaModified`.
- Keep `EnumScenarioName.ApophisElongated` as an independent scenario.
- Add `EnumScenarioName.Arrokoth`, `EnumScenarioName.Comet67P`, and `EnumScenarioName.Toutatis`.
- Remove `bLoadModifiedVariant` from `DefineShapeModel`.
- Old `ItokawaModified`/`ModifiedItokawa` names fail with `CScenarioRegistry:UnsupportedScenario`.

- [ ] Add failing registry tests for new scenario aliases and removed `ItokawaModified`.
- [ ] Add failing builder test proving `bLoadModifiedVariant` is no longer accepted and explicit `ApophisElongated` remains valid.
- [ ] Update enum and registry resolution.
- [ ] Remove Itokawa modified branch/path metadata from builder and tests.
- [ ] Keep Apophis and ApophisElongated dispatch explicit, not flag-driven.
- [ ] Run registry and shape-model tests.

## Task 4: SimGears Data Root And Manifests

**Files:**
- Create: `data/.gitignore`
- Create: `data/common/spice/manifest.json`
- Create: `data/scenarios/<scenario>/manifest.json` for `Apophis`, `ApophisElongated`, `Itokawa`, `Bennu`, `Didymos`, `Eros`, `Arrokoth`, `Comet67P`, and `Toutatis`
- Create: `matlab/simulation_management/ResolveSimGearsDataRoot.m`
- Create: `matlab/simulation_management/LoadScenarioDataManifest.m`
- Create: `tests/matlab/simulation_management/testScenarioDataManifests.m`
- Modify: `CScenarioRegistry` to expose manifest relative paths and default asset relative paths.

**Interfaces:**
- `ResolveSimGearsDataRoot()` returns explicit override, `SIMGEARS_DATA_ROOT`, or repo `data/`.
- `LoadScenarioDataManifest(enumOrName)` returns decoded manifest and validates schema.
- Manifests are tracked metadata; asset folders are ignored.

- [ ] Add failing tests for SimGears-first data root, manifest loading, unknown scenario, missing required manifest fields, and no nav-backend fallback.
- [ ] Add `.gitignore` for fetched assets and derived outputs while keeping manifests tracked.
- [ ] Add manifest files with source URLs/provenance and shape-fidelity metadata.
- [ ] Add manifest loader and data-root resolver.
- [ ] Update registry specs to include manifest paths and default shape asset path fields.
- [ ] Run manifest and registry tests.

## Task 5: Transition Existing nav-backend Data Ownership

**Files:**
- Create ignored SimGears asset folders under `data/scenarios/.../assets/...`
- Copy/move local nav-backend scenario assets into SimGears working tree as ignored fetched assets when present.
- Modify: `DefineShapeModel` path resolution.
- Modify: `CScenarioRegistry` default shape paths.
- Test: `tests/matlab/simulation_models/testCShapeModelSimplifyMesh.m`

**Interfaces:**
- SimGears no longer resolves `WS_NAVSYS/nav-backend/data/SPICE_kernels` as an implicit fallback.
- Existing explicit `charDataRootPath` remains supported.
- Missing required shape assets fail fast with a message naming the SimGears manifest/fetch path.

- [ ] Add failing test that an empty `charDataRootPath` resolves through SimGears data and not nav-backend.
- [ ] Replace `ResolveNavBackendDataRoot_` with SimGears data-root resolution.
- [ ] Transition local Itokawa/Bennu/Didymos relevant assets into ignored SimGears `data/` folders if available locally.
- [ ] Update builder defaults to use registry manifest/default asset paths.
- [ ] Run shape-model focused tests.

## Task 6: Fetch/Validate Tool And Shape-Fidelity Selection

**Files:**
- Create: `tools/data/FetchScenarioData.m`
- Create: `tools/data/ValidateScenarioDataManifest.m`
- Create/modify: `tests/matlab/simulation_management/testScenarioDataManifests.m`

**Interfaces:**
- `ValidateScenarioDataManifest(strManifest, options)` validates required fields, scenario name, asset records, size policy, hashes when present, and default shape selection.
- `FetchScenarioData(enumOrName, options)` supports dry-run/local validation; actual download is best-effort and not required for unit tests.
- Shape defaults prefer max practical fidelity `< 1 GB`; albedo/spectral entries can be recorded but not required for shape-runnable status.

- [ ] Add failing fixture tests for dry-run validation, checksum mismatch, missing default shape asset, and oversized default shape without explicit override.
- [ ] Implement manifest validator.
- [ ] Implement fetcher with dry-run and local-only validation first.
- [ ] Run manifest/fetcher tests.

## Task 7: New Runnable Tagged Scenarios

**Files:**
- Modify: `CScenarioRegistry`
- Modify: `DefineShapeModel`
- Modify: `CScenarioGenerator` only where scenario metadata is consumed.
- Modify: scenario manifests.
- Test: `testCScenarioRegistry`, `testCShapeModelSimplifyMesh`, `testScenarioDataManifests`.

**Interfaces:**
- `Arrokoth`: PDS New Horizons shape + albedo metadata; shape confidence high.
- `Comet67P`: Rosetta shape bundle; texture/spectral processing marked follow-up.
- `Toutatis`: JPL/PDS radar shape; texture procedural/synthetic.
- `Eros`: Gaskell full-resolution source default when practical, NAIF Q=64 fast/test fallback, MSI albedo metadata.

- [ ] Add tests for scenario tags, confidence, source URLs, and default shape selection.
- [ ] Add registry metadata for the four scenarios.
- [ ] Update builders to allow shape-runnable mode with explicit or manifest-resolved shape assets.
- [ ] Run registry, manifest, and builder smoke tests.

## Task 8: Final Verification And Commit Split Review

**Files:**
- All touched files from tasks above.

**Interfaces:**
- No commits.
- Staging state may be left as-is unless the user asks to stage; report any pre-existing staged files separately from unstaged implementation.

- [ ] Run focused verification:
  - `runtests('tests/matlab/general_utils/datastructs/testEnumLengthUnits')`
  - `runtests('tests/matlab/simulation_management/testCScenarioRegistry')`
  - `runtests('tests/matlab/simulation_management/testScenarioDataManifests')`
  - `runtests('tests/matlab/simulation_models/accelerations/testSSphericalHarmonicsGravityData')`
  - `runtests('tests/matlab/simulation_models/testCShapeModelSimplifyMesh')`
- [ ] Run `git diff --check` and `git diff --cached --check`.
- [ ] Review final dirty tree and recommend commit split:
  - SH enum/file-backed generator cleanup.
  - First-class scenario identity cleanup.
  - SimGears data layout/manifests/fetch validation.
  - New tagged scenarios and Eros/Apophis harmonization.
  - Existing unrelated dynamics/noise/photometry/tooling groups.

## Task 9: Registry-Owned Scenario SPICE Environments

**Files:**
- Modify: `matlab/simulation_management/CScenarioRegistry.m`
- Modify: `matlab/simulation_management/CSPICEkerLoader.m`
- Modify: `data/scenarios/Apophis/manifest.json`
- Create: ignored environment assets under `data/scenarios/Apophis/assets/spice/`
- Create/modify: focused simulation-management tests

**Interfaces:**
- A scenario may declare `charDefaultSpiceMetaKernelRelativePath` in its registry spec.
- `CSPICEkerLoader` prefers that SimulationGears-owned metakernel and does not require a legacy kernel root for the scenario.
- Scenarios without a registered metakernel retain the existing explicit legacy-root behavior.
- Scenario environment packs contain target and generic-body kernels only; spacecraft trajectory kernels remain outside the scenario database.

- [x] Add failing tests for registry metadata, manifest alignment, and data-root precedence.
- [x] Add the registry metakernel field and SimulationGears-first loader path.
- [x] Copy the generic Apophis environment pack into the ignored scenario asset tree and record its hashes in the manifest.
- [x] Verify the copied hashes and load the real environment with MICE.
- [x] Verify target-fixed attitude and target/Sun ephemerides at the pilot epoch.
- [x] Run the COSMICA Apophis truth-setup boundary without an external Apophis initializer or trajectory kernel.

## Task 10: Complete Legacy SPICE Data Ownership Migration

**Files:**
- Modify: scenario and common-data manifests under `data/`
- Create: ignored scenario, common, and mission SPICE asset packs under `data/`
- Modify: registry and loader contracts where additional pack types require explicit ownership
- Modify: consumers in their owning repositories through separately reviewed batches

**Interfaces:**
- Generic NAIF kernels are owned by the SimulationGears common-data collection.
- Target environment kernels are owned by registered scenario packs.
- Spacecraft trajectories, spacecraft attitude, and mission frames are owned by explicit mission packs rather than target scenarios.
- The legacy nav-backend kernel root remains available until all consumers have migrated and passed runtime validation.

- [ ] Inventory every legacy kernel asset and classify it as common, scenario, or mission data.
- [ ] Define manifest-backed common and mission-pack contracts without weakening scenario ownership.
- [ ] Copy classified assets into ignored SimulationGears data folders and record hashes and provenance.
- [ ] Migrate each scenario and mission consumer in its owning repository through a separate review batch.
- [ ] Verify representative frame, attitude, and ephemeris queries for every migrated pack.
- [ ] Confirm no runtime consumer resolves the legacy nav-backend kernel root.
- [ ] Create and verify a recoverable archive of the legacy kernel directory.
- [ ] Remove the legacy directory only after explicit user approval.

## Task 11: Embedded Apophis Degree-16 Gravity Family

**Files:**
- Create: `examples/matlab/shape_models/GenerateScenarioSHGravityCoefficients.m`
- Create: `matlab/simulation_models/accelerations/RescaleSphericalHarmonicsReferenceRadius.m`
- Modify: `examples/matlab/shape_models/DemoItokawaDegree16GravityComparison.m`
- Modify: `matlab/simulation_management/CScenarioRegistry.m`
- Modify: `tests/matlab/simulation_management/testCScenarioRegistry.m`
- Modify: `tests/matlab/simulation_models/accelerations/testSSphericalHarmonicsGravityData.m`

**Interfaces:**
- `GenerateScenarioSHGravityCoefficients(...)` is manually runnable for any registry-backed target
  with an installed shape and finite gravity metadata. It reuses `DefineShapeModel`, centers the uniform-density
  volume centroid, fits outside the enclosing sphere, and rescales the generated rows to the target's registered
  mean/reference normalization radius.
- The generator returns and optionally prints the MATLAB-ready unnormalized coefficient rows used for registry
  embedding. The mesh enclosing radius remains generation evidence rather than replacing the physical mean radius.
- `CScenarioRegistry.GetSphericalHarmonicsGravityData("Apophis", degree, "km")` returns exact prefixes of the
  accepted embedded degree-16 family.
- Normal runtime setup consumes only the registry and never performs an online fit.
- Apophis and Itokawa generation reuse one shared reference-radius transformation.

- [x] Add a failing structural test for unavailable Apophis registry coefficients.
- [x] Implement the generic registry-target coefficient generator and reuse the promoted Itokawa radius transform.
- [x] Generate the degree-16 family from the registered full mesh and review the fit evidence. The complete
  3,996-face/2,000-vertex mesh was translated from a 3.511e-7 km volume-COM offset to a 5.544e-18 km residual;
  one full-resolution fit iteration produced 151 finite rows with perturbative validation RMS errors of 2.876e-2
  for acceleration and 9.050e-3 for potential.
- [x] Embed the accepted coefficient rows and physical metadata in `CScenarioRegistry`. The coefficient family is
  normalized at the registered 0.175930344 km mean radius; the 0.248137567 km enclosing mesh radius remains fit
  evidence and is not substituted for the physical reference radius.
- [x] Verify exact degree-prefix behavior and dimensional scaling. Registry and shared reference-radius tests pass,
  and numerical evaluation before/after radius rescaling agrees to 3.46e-16 relative acceleration and 9.28e-17
  relative potential.
- [x] Re-run the COSMICA Apophis truth-setup boundary with registry-backed spherical harmonics. The real typed
  one-day condition loads the generic Apophis SPICE pack and degree-16 family, producing 151 SH rows with
  `dRefRadius = 0.175930344 km` and spherical-harmonics truth enabled.
