# Merge-Blocking Compatibility Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. User constraint: do not commit; preserve unrelated work, stage only the verified merge-blocker batch, and report the proposed commit message.

**Goal:** Remove the three remaining merge blockers without changing active-burn physics, substituting a different Toutatis mesh, or changing explicitly enabled distance-scaled SRP behavior.

**Architecture:** Restore each compatibility contract at its owning decision point. The finite-burn builder selects the inactive branch before requiring positive thrust. Scenario asset fetching declares and verifies the downloaded payload format independently of the source filename extension. Max-fidelity dynamics resolves SRP pressure mode with precedence `explicit model flag > payload flag > legacy fixed-pressure default`.

**Tech Stack:** MATLAB R2023b, `matlab.unittest`, codegen-compatible MATLAB structs/functions, Python 3 `unittest`, tracked JSON scenario manifests, and the existing scenario-asset fetch CLI.

## Execution Status — 18-07-2026

- [x] Finite-burn compatibility implemented with RED `4/5` and GREEN `5/5` focused evidence.
- [x] Toutatis manifest/fetch validation implemented with GREEN `8/8` Python evidence and a clean official PDS fetch of `2,879,779` bytes at SHA-256 `28c94e3c5c4fadab97c6f1fc78dbb3800a4248b86302681cf1c629e48df5f00a`.
- [x] Fixed-pressure SRP runtime default and precedence regressions implemented with GREEN `9/9` focused RHS/Jacobian evidence.
- [x] Combined controller verification passed: `26/26` MATLAB tests, `8/8` Python tests, live Toutatis fetch/verify, and `git diff --check`.
- [ ] Representative MEX verification remains blocked before Jacobian generation because `BuildMexTargets_InertialDynMaxFidelity` omits `strStochasticAccelData`; its explicit recomputation flag would also bypass the changed default. Repair and add a field-minimal fixed-pressure specialization before merge.
- [x] Final whole-batch review found no implementation defect and approved the seven implementation/test files for staging; merge remains gated on the MEX item above.

## Fixed Design Decisions

- [ ] Treat `norm(dDeltaV_IN) < eps('single')` as the existing inactive-burn contract. Zero thrust is valid only in that branch; a nonzero requested DeltaV still raises `ComputeFiniteBurnFromDeltaV:InvalidThrust`.
- [ ] Preserve the PDS `4179toutatis2` mesh. The official `4179toutatis2.tab` payload is already Wavefront OBJ syntax and is byte-identical to the currently installed ignored `4179toutatis2.obj` (`2,879,779` bytes; SHA-256 `28c94e3c5c4fadab97c6f1fc78dbb3800a4248b86302681cf1c629e48df5f00a`). Its [PDS label](https://sbnarchive.psi.edu/pds4/non_mission/compil.ast.radar.shape-models/data/4179toutatis2.xml) explicitly says the format is identical to Wavefront OBJ. Do not replace it with JPL's separately published `hirestoutatis.obj`, whose mesh and checksum differ.
- [ ] Make distance-based SRP pressure opt-in. The default is fixed `strSRPdata.dP_SRP`; `strModelConfigFlags.bRecomputeSRPpressureFromDistance` remains the highest-priority override, followed by `strSRPdata.bRecomputePressureFromDistance`.
- [ ] Do not modify or stage the pre-existing dirty submodule state under `lib/MathCore_for_ComputerVision` or the untracked imported `.github/pull_request_template.md`.

---

## Task 1: Restore the Zero-DeltaV Inactive-Burn Contract

**Files:**

- Modify: `tests/matlab/simulation_models/accelerations/testFiniteBurnThrustModel.m`
- Modify: `matlab/simulation_models/accelerations/ComputeFiniteBurnFromDeltaV.m`

**Behavioral contract:**

- `DeltaV == 0`, `thrust == 0`: return an inactive, zero-duration, zero-propellant profile.
- active `DeltaV`, `thrust == 0`: fail with `ComputeFiniteBurnFromDeltaV:InvalidThrust` before any mass-flow division.
- active `DeltaV`, `thrust > 0`: preserve current rocket-equation results and length-unit behavior.

- [ ] Keep the existing red regression `testZeroDeltaVProducesInactiveBurn` unchanged; it already expresses the desired inactive profile and currently fails at the unconditional thrust assertion.
- [ ] Add `testNonzeroDeltaVRequiresPositiveThrust` using a clearly active request such as `[0.0; 0.012; 0.0]` and verify the exact error ID `ComputeFiniteBurnFromDeltaV:InvalidThrust`.
- [ ] Run the focused file before implementation and record that only the zero-DeltaV case is expected to fail:

  ```bash
  matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath('matlab'); SetupSimGears; results = runtests('tests/matlab/simulation_models/accelerations/testFiniteBurnThrustModel.m'); disp(results);"
  ```

- [ ] Move the `dThrust > 0.0` assertion from before the `dDeltaVNorm` branch into the active-burn `else` branch, immediately before the rocket-equation/mass-flow calculations. Do not weaken the assertion and do not special-case the later division.
- [ ] Add a `18-07-2026` changelog entry stating that zero-DeltaV profiles permit zero thrust while active burns still require positive thrust.
- [ ] Re-run the focused test and require every result to pass:

  ```bash
  matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath('matlab'); SetupSimGears; results = runtests('tests/matlab/simulation_models/accelerations/testFiniteBurnThrustModel.m'); disp(results); assert(all([results.Passed]));"
  ```

**Task gate:** No active-burn calculation changes, and both zero/zero acceptance and nonzero/zero rejection are covered.

---

## Task 2: Make the Toutatis Source-to-Local Format Contract Explicit

**Files:**

- Modify: `tests/python/test_fetch_scenario_assets.py`
- Modify: `tools/data/fetch_scenario_assets.py`
- Modify: `data/scenarios/Toutatis/manifest.json`

**Manifest contract:**

```json
{
  "local_path": "scenarios/Toutatis/assets/shape/4179toutatis2.obj",
  "source_url": "https://sbnarchive.psi.edu/pds4/non_mission/compil.ast.radar.shape-models/data/4179toutatis2.xml",
  "download_url": "https://sbnarchive.psi.edu/pds4/non_mission/compil.ast.radar.shape-models/data/4179toutatis2.tab",
  "content_format": "wavefront_obj",
  "sha256": "28c94e3c5c4fadab97c6f1fc78dbb3800a4248b86302681cf1c629e48df5f00a",
  "size_gb": 0.002879779,
  "fidelity": "pds_high_resolution_radar_wavefront_obj"
}
```

- [ ] Extend the test asset helper with optional `content_format` and `sha256` arguments without making `content_format` mandatory for unrelated existing manifests.
- [ ] Add a failing `test_wavefront_obj_content_format_rejects_non_obj_download` fixture. Serve a local `.tab` URI containing non-OBJ table text, declare `content_format: wavefront_obj`, run a real CLI fetch into an absent `.obj` destination, and assert a nonzero return code plus no installed destination file.
- [ ] Add `test_tab_named_wavefront_obj_download_is_installed_as_obj`. Serve a local `.tab` URI containing at least three `v` records and one `f` record, pin its checksum, fetch it, and assert the `.obj` destination is byte-identical.
- [ ] Add `test_tracked_toutatis_manifest_pins_pds_wavefront_obj`, asserting the tracked asset has `content_format == 'wavefront_obj'`, the exact checksum above, the `.tab` source URL, and the `.obj` local path. This documents that the extension change is intentional rather than accidental.
- [ ] Run the Python tests before implementation; the invalid-payload and tracked-manifest tests must fail:

  ```bash
  python3 -m unittest tests/python/test_fetch_scenario_assets.py
  ```

- [ ] In `validate_manifest_schema(...)`, accept absent `content_format` for backward compatibility, accept `wavefront_obj` when present, and reject unsupported declared values with the manifest path and asset ID in the error.
- [ ] Add one narrow content verifier for `wavefront_obj` that reads UTF-8 text and requires at least one vertex record (`v`) and one face record (`f`). Report the asset ID, candidate path, declared format, and manifest path on failure.
- [ ] Extend checksum verification to accept an explicit candidate path so a temporary download can be checked before installation.
- [ ] Apply checksum and declared-content verification in all three paths: `--verify-only`, an already-existing non-overwritten asset, and a newly fetched asset.
- [ ] In `fetch_direct_file(...)`, verify the temporary file before `Path.replace(...)`; an invalid payload must never become the final `.obj`.
- [ ] Update the Toutatis manifest exactly as shown above. Keep the official PDS `.tab` URL because it is the intended mesh and is semantically OBJ; do not swap in a different JPL model merely to obtain an `.obj` URL suffix.
- [ ] Re-run the Python test file and require all tests to pass:

  ```bash
  python3 -m unittest tests/python/test_fetch_scenario_assets.py
  ```

- [ ] Perform one clean, network-backed fetch and verification outside the repository data tree:

  ```bash
  toutatis_fetch_root=$(mktemp -d)
  mkdir -p "$toutatis_fetch_root/scenarios/Toutatis"
  cp data/scenarios/Toutatis/manifest.json "$toutatis_fetch_root/scenarios/Toutatis/manifest.json"
  python3 tools/data/fetch_scenario_assets.py --data-root "$toutatis_fetch_root" --scenario Toutatis
  python3 tools/data/fetch_scenario_assets.py --data-root "$toutatis_fetch_root" --scenario Toutatis --verify-only
  sha256sum "$toutatis_fetch_root/scenarios/Toutatis/assets/shape/4179toutatis2.obj"
  ```

  Expected checksum: `28c94e3c5c4fadab97c6f1fc78dbb3800a4248b86302681cf1c629e48df5f00a`.

**Task gate:** A clean fetch preserves the exact PDS mesh, explicitly validates it as Wavefront OBJ, and cannot install arbitrary `.tab` content under an `.obj` name.

---

## Task 3: Restore Fixed-Pressure SRP as the Legacy Default

**Files:**

- Modify: `tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/private/ResolveInertialDynMaxFidelityConfig.m`
- Verify: `tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP.m`

**Resolution precedence:**

1. If `strModelConfigFlags.bRecomputeSRPpressureFromDistance` exists, use it.
2. Else if `strDynParams.strSRPdata.bRecomputePressureFromDistance` exists, use it.
3. Else use `false` and consume fixed `strDynParams.strSRPdata.dP_SRP`.

- [ ] Add `testLegacyFixedPressureSRPDefaultsWithoutReferenceDistance` using an SRP payload containing only `dP_SRP = 4.0`; omit `dP_SRP0`, `dReferenceDistance`, and `bRecomputePressureFromDistance`.
- [ ] Disable main/third-body gravity, spherical harmonics, polyhedron gravity, panel SRP, and eclipse while retaining the Sun ephemeris needed for the cannonball direction. At state `[1; 0; 0; 0; 0; 0]` with the existing fixture geometry, assert acceleration `[-8; 0; 0]`.
- [ ] In the same regression, pass the RHS diagnostics into `evalJac_InertialDynMaxFidelity(...)` and assert the fixed-pressure SRP position block is `diag([0.0, 0.8, 0.8])`. This proves both public max-fidelity entry points accept the legacy payload and use the fixed-pressure derivative.
- [ ] Run the new regression before implementation and record the missing `dP_SRP0`/`dReferenceDistance` failure caused by the current implicit `true` default:

  ```bash
  matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath('matlab'); SetupSimGears; results = runtests('tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m'); disp(results);"
  ```

- [ ] Change only the resolver initialization from `true` to `false`. Preserve the existing explicit-model-flag branch and payload-flag fallback unchanged.
- [ ] Add a `18-07-2026` changelog entry documenting fixed pressure as the compatibility default and distance recomputation as opt-in.
- [ ] Remove the explicit recomputation field from `testCannonballSRPDistanceScalingAndEclipse` while retaining the fixture payload's `bRecomputePressureFromDistance = true`; its near/far `-8/-2` assertions then prove the payload-level opt-in still works.
- [ ] In `testPanelSRPSelectedOnlyWhenPanelDataExists`, set the payload flag to `false` while retaining the explicit model flag `bRecomputeSRPpressureFromDistance = true`; this proves the explicit model configuration remains the highest-priority override.
- [ ] Re-run the RHS and analytical panel-SRP Jacobian suites:

  ```bash
  matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath('matlab'); SetupSimGears; files = {'tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m', 'tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP.m'}; results = runtests(files); disp(results); assert(all([results.Passed]));"
  ```

- [ ] Rebuild the representative max-fidelity RHS/Jacobian MEX targets so the changed compile-time default is codegen-verified:

  ```bash
  matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath('matlab'); SetupSimGears; buildDir = tempname; mkdir(buildDir); info = BuildMexTargets_InertialDynMaxFidelity(buildDir); assert(numel(info.cellMexTargets) == 2);"
  ```

**Task gate:** Field-minimal legacy payloads run through RHS and Jacobian, while both explicit and payload-level recomputation opt-ins retain their current precedence and behavior.

---

## Task 4: Combined Merge Gate and Review Boundaries

**Files:**

- Review all files changed in Tasks 1-3.
- Do not commit. Stage only the plan and verified Task 1-3 files after every merge gate passes.

- [ ] Run the complete focused merge-blocker set in one MATLAB process:

  ```bash
  matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath('matlab'); SetupSimGears; files = {'tests/matlab/simulation_models/accelerations/testFiniteBurnThrustModel.m', 'tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m', 'tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP.m', 'tests/matlab/simulation_management/testScenarioDataManifests.m'}; results = runtests(files); disp(results); assert(all([results.Passed]));"
  ```

- [ ] Run the Python fetcher suite:

  ```bash
  python3 -m unittest tests/python/test_fetch_scenario_assets.py
  ```

- [ ] Run whitespace and repository-state checks:

  ```bash
  git diff --check
  git status --short
  ```

- [ ] Inspect the exact implementation diff separately from pre-existing workspace changes:

  ```bash
  git diff -- matlab/simulation_models/accelerations/ComputeFiniteBurnFromDeltaV.m tests/matlab/simulation_models/accelerations/testFiniteBurnThrustModel.m
  git diff -- data/scenarios/Toutatis/manifest.json tools/data/fetch_scenario_assets.py tests/python/test_fetch_scenario_assets.py
  git diff -- matlab/simulation_models/dynamics/private/ResolveInertialDynMaxFidelityConfig.m tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m
  ```

- [ ] Re-read every changed test and implementation together. Reject the batch if a test only proves execution rather than the exact compatibility contract, if Toutatis geometry changes, or if an explicit SRP recomputation request stops winning over the default.
- [ ] Refresh the PR description's test evidence only after every gate above is green. Keep tags/releases separate from these fixes; the proposed historical tag split should be applied only after the merge-blocker diff is reviewed and accepted.

## Suggested Staged Commit

No commit is performed by this plan. The verified merge-blocker batch is staged as one review unit with suggested subject:

`Fix remaining develop merge blockers`

Its three functional components remain explicit in the commit body or PR description:

- allow zero thrust for inactive finite-burn profiles;
- validate and pin the Toutatis PDS payload as Wavefront OBJ;
- restore the fixed-pressure SRP compatibility default.

## Completion Criteria

- [ ] Zero-DeltaV/zero-thrust produces an inactive finite burn; active DeltaV/zero-thrust still errors.
- [ ] A fresh Toutatis fetch installs the pinned PDS model as a semantically verified OBJ, with no geometry substitution.
- [ ] Max-fidelity RHS and Jacobian accept fixed-pressure payloads lacking `dReferenceDistance` by default.
- [ ] Explicit distance recomputation remains green in RHS, Jacobian, and MEX codegen coverage.
- [ ] Focused MATLAB/Python suites, live asset verification, `git diff --check`, and manual diff review all pass.
- [ ] No unrelated submodule, template, staging, or commit state is altered.
