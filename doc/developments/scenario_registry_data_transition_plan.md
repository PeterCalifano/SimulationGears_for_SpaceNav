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
