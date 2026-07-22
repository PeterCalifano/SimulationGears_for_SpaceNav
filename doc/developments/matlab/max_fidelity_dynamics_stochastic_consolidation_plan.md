# Max-Fidelity Dynamics Stochastic Consolidation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans or an equivalent staged review workflow to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. User constraint: do not commit; stage only when explicitly requested.

**Goal:** Consolidate the current max-fidelity dynamics implementation and replace the single generic stochastic residual hook with two physically scoped truth-dynamics channels: gravity-related residuals in RTN and SRP-related residuals in an illumination/SRP frame.

**Architecture:** Keep deterministic force-model selection in `ResolveInertialDynMaxFidelityConfig(...)`, evaluate deterministic selected forces first, then inject source-specific colored residuals scaled by the selected deterministic acceleration norms. Stochastic profiles remain pre-generated and deterministic at RHS call time; no random draw is allowed inside RHS or Jacobian evaluation.

**Tech Stack:** MATLAB R2023b, `matlab.unittest`, codegen-compatible structs/functions, existing max-fidelity RHS/Jacobian entry points, existing Gauss-Markov profile generator/evaluator.

## Global Constraints

- Do not commit.
- Preserve unrelated staged and unstaged work.
- Use `apply_patch` for manual edits.
- Keep stochastic truth perturbations separate from estimator process-noise or DMC concepts.
- Fail early with clear error IDs for inconsistent deterministic model selection or singular stochastic-frame geometry.
- Keep runtime stochastic evaluation deterministic; randomness belongs only in profile/data generation.
- Add public function documentation headers and changelog entries using the current MATLAB file convention.
- Keep tests focused and non-superfluous; include edge cases and finite-difference validation where Jacobians are touched.

---

## Current Open Changes Included In This Plan

- [ ] **Staged SRP/Jacobian candidate:** `matlab/simulation_models/accelerations/EvalJac_QuadsModelSRP.m`, staged changes in `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`, `tests/matlab/simulation_models/accelerations/testComputeQuadsModelSRP.m`, and `tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP.m` are the current analytical panel-SRP Jacobian batch.
- [ ] **Staged settings support:** `matlab/general_utils/datastructs/CBaseSettings.m` is staged but not obviously part of this dynamics batch; review before including it in any stage.
- [ ] **Unstaged stochastic edit:** `matlab/simulation_models/accelerations/EvalGaussMarkovAccel.m` has an unstaged edit and must be reviewed before changing the stochastic data contract.
- [ ] **Unstaged Jacobian edit:** `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m` has staged plus unstaged changes; inspect with cached and non-cached diffs before editing.
- [ ] **Untracked adjacent dynamics work:** finite-burn, RHS, SRP, photometry, propagator, generator, and `tools/gravity/` files are present but are not automatically part of this plan unless a task below names them explicitly.

## Current Direction

- [ ] Keep the panel-SRP analytical Jacobian work as the first review candidate, because it is already staged and has focused tests.
- [ ] Consolidate max-fidelity RHS/Jacobian structure before adding new stochastic behavior, so exclusive deterministic model selection is unambiguous.
- [ ] Replace the current single `strStochasticAccelData` inertial acceleration profile with source-specific channels while keeping a narrow legacy compatibility path during transition.
- [ ] Add two stochastic truth channels:
  - gravity residual in RTN, scaled by selected deterministic target-gravity magnitude;
  - SRP residual in an SRP/illumination frame, scaled by selected deterministic SRP magnitude.

## Design Decisions To Implement

- [ ] Use a dimensionless unit Gauss-Markov profile for new channels. The RHS scales it by acceleration magnitude so default amplitudes are relative and unit-safe.
- [ ] Default correlation time is `6 hours` converted by the scenario time-unit convention used by the caller.
- [ ] Default randomized three-sigma relative amplitude is between `0.01%` and `0.5%` of the selected deterministic source acceleration magnitude. This randomization must happen while building the profile/config, not inside RHS.
- [ ] Gravity stochastic scale source is the selected target-gravity acceleration only: central gravity plus active non-spherical model, or full polyhedron gravity when polyhedron is selected. Third-body and SRP accelerations are excluded.
- [ ] SRP stochastic scale source is the selected deterministic SRP acceleration only. If SRP is disabled, missing, or eclipsed, the SRP stochastic acceleration and Jacobian are exactly zero.
- [ ] The stochastic channels do not make mutually exclusive deterministic models additive. They only perturb the deterministic model that has already been selected.
- [ ] Exact Jacobian support should include the scale derivative and frame derivative for the active stochastic channels. If a singular frame prevents a mathematically valid derivative, fail fast instead of silently returning a zero partial.

## Task 1: Baseline Review And Commit-Boundary Cleanup

**Files:**
- Read: `matlab/simulation_models/accelerations/EvalJac_QuadsModelSRP.m`
- Read: `matlab/simulation_models/accelerations/EvalGaussMarkovAccel.m`
- Read: `matlab/simulation_models/dynamics/evalRHS_InertialDynMaxFidelity.m`
- Read: `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- Read: `matlab/simulation_models/dynamics/private/ResolveInertialDynMaxFidelityConfig.m`
- Test: `tests/matlab/simulation_models/accelerations/testComputeQuadsModelSRP.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m`

**Interfaces:**
- Produces a clean review boundary for the current staged panel-SRP Jacobian work.
- Produces a written decision on whether `CBaseSettings.m` belongs to this dynamics commit stream.

- [ ] Inspect cached and unstaged diffs separately:
  - `git diff --cached -- matlab/simulation_models/accelerations/EvalJac_QuadsModelSRP.m matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m tests/matlab/simulation_models/accelerations/testComputeQuadsModelSRP.m tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP.m`
  - `git diff -- matlab/simulation_models/accelerations/EvalGaussMarkovAccel.m matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- [ ] Run focused baseline tests for the currently staged Jacobian candidate:
  - `runtests('tests/matlab/simulation_models/accelerations/testComputeQuadsModelSRP')`
  - `runtests('tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP')`
- [ ] Record any failing test as either pre-existing environment setup, staged-regression blocker, or unrelated dirty-tree interaction before editing.

## Task 2: Deterministic Force-Model Selection Must Be Exclusive

**Files:**
- Modify: `matlab/simulation_models/dynamics/private/ResolveInertialDynMaxFidelityConfig.m`
- Modify: `matlab/simulation_models/dynamics/evalRHS_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m`

**Interfaces:**
- Produce `strModelConfig.ui8SelectedGravityModel` with values:
  - `uint8(0)`: no target gravity;
  - `uint8(1)`: central only;
  - `uint8(2)`: spherical harmonics;
  - `uint8(3)`: polyhedron.
- Produce `strModelConfig.ui8SelectedSRPModel` with values:
  - `uint8(0)`: no SRP;
  - `uint8(1)`: cannonball;
  - `uint8(2)`: panel.
- Preserve existing flag compatibility where possible, but never evaluate cannonball and panel SRP together or spherical harmonics and polyhedron gravity together.

- [x] Add a failing test that non-empty SH data plus non-empty polyhedron data with both include flags enabled errors with `ResolveInertialDynMaxFidelityConfig:ConflictingGravityModels`.
- [ ] Add a failing test that an explicit panel SRP request without panel data errors with `ResolveInertialDynMaxFidelityConfig:MissingPanelSRPData`.
- [ ] Add a failing test that an explicit cannonball SRP request without cannonball scalar data errors with `ResolveInertialDynMaxFidelityConfig:MissingCannonballSRPData`.
- [x] Update `ResolveInertialDynMaxFidelityConfig(...)` to compute `ui8SelectedGravityModel` once.
- [ ] Update `ResolveInertialDynMaxFidelityConfig(...)` to compute `ui8SelectedSRPModel` once.
- [x] Update RHS and Jacobian to branch only on `ui8SelectedGravityModel` for target gravity.
- [ ] Update RHS and Jacobian to branch only on `ui8SelectedSRPModel` for SRP.
- [ ] Keep diagnostic fields such as `bCannonballSRPSelected` and `bPanelSRPSelected`, but derive them from `ui8SelectedSRPModel`.
- [x] Run `testEvalRHS_InertialDynMaxFidelity` and the panel-SRP Jacobian test.
- [x] Build separate compile-time SH and polyhedron RHS/Jacobian MEX variants from one payload containing both datasets.

## Task 3: Extract Shared Geometry Helpers

**Files:**
- Create: `matlab/simulation_models/dynamics/private/ResolveSCQuaternion.m`
- Create: `matlab/simulation_models/accelerations/ComputePolyhedronGravityJacobianCorrection.m`
- Modify: `matlab/simulation_models/dynamics/evalRHS_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m`

**Interfaces:**
- `ResolveSCQuaternion(strDynParams)` returns `dqSCBwrtIN` and owns the spacecraft-attitude default/fallback rule used by RHS and Jacobian.
- `ComputePolyhedronGravityJacobianCorrection(dPosSC_IN, dDCMmainAtt_INfromTF, dMainGM, strPolyhedronGravityData)` returns the inertial-frame polyhedron-over-central gravity Jacobian correction.

- [ ] Move local `ResolveSCQuaternion_` to the dedicated private function without changing behavior.
- [ ] Move local `ComputePolyhedronJacobianCorrection_` to the acceleration helper without changing behavior.
- [ ] Add headers/changelog entries to the new public/shared functions.
- [ ] Run focused RHS/Jacobian tests to prove the extraction is behavior-preserving.

## Task 4: Stochastic Channel Data Contract

**Files:**
- Modify: `matlab/simulation_models/accelerations/GenerateGaussMarkovAccelSeq.m`
- Modify: `matlab/simulation_models/accelerations/EvalGaussMarkovAccel.m`
- Create: `matlab/simulation_models/accelerations/BuildStochasticTruthAccelChannel.m`
- Test: `tests/matlab/simulation_models/accelerations/testGaussMarkovAccelSeq.m`

**Interfaces:**
- Existing absolute inertial profiles remain readable during transition.
- New source-specific channels use a dimensionless profile payload:
  - `charChannelName`: `'gravity_rtn'` or `'srp'`;
  - `charFrame`: `'RTN'`, `'SRP_SUNLINE'`, or `'IN'` for legacy profiles;
  - `dTimeGrid`, `dUnitGrid`, `dStandardNormalInnovation`, `dSigmaUnit`, `dTimeConst`, `dMeanUnit`, `dTimeStep`, `ui32Seed`;
  - `dThreeSigmaRelativeScale`;
  - `charScaleSource`: `'selected_target_gravity_norm'` or `'selected_srp_norm'`.
- `BuildStochasticTruthAccelChannel(...)` builds channel structs and performs default three-sigma relative-scale randomization outside RHS.

- [ ] Add tests that a dimensionless profile is repeatable for a fixed seed and stores the requested frame/channel labels.
- [ ] Add tests that default three-sigma relative scale lies in `[1.0e-4, 5.0e-3]` and is repeatable for a fixed seed.
- [ ] Add tests that old absolute `dAccelGrid` profiles still evaluate through the legacy path.
- [ ] Keep `EvalGaussMarkovAccel(...)` deterministic and free of random draws.

## Task 5: Gravity-Related Stochastic Residual In RTN

**Files:**
- Create: `matlab/simulation_models/accelerations/EvalStochasticGravityRTNAccel.m`
- Create: `matlab/simulation_models/accelerations/EvalJac_StochasticGravityRTNAccel.m`
- Modify: `matlab/simulation_models/dynamics/evalRHS_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- Test: `tests/matlab/simulation_models/accelerations/testStochasticGravityRTNAccel.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityStochastic.m`

**Interfaces:**
- `EvalStochasticGravityRTNAccel(dEvalTime, dxOrbitState_IN, dAccGravity_IN, strGravityChannel)` returns `dAccStochasticGravity_IN`.
- `EvalJac_StochasticGravityRTNAccel(dxOrbitState_IN, dAccGravity_IN, dJacGravity_IN, dUnitRTN)` returns a `3x6` orbit-state partial.
- RTN basis convention:
  - `R = r / norm(r)`;
  - `N = cross(r, v) / norm(cross(r, v))`;
  - `T = cross(N, R)`;
  - `dDCM_INfromRTN = [R, T, N]`.

- [ ] Add tests for exact RTN basis orientation on a simple circular state.
- [ ] Add tests that gravity stochastic acceleration scales with `norm(dAccGravity_IN)` and not with third-body or SRP acceleration.
- [ ] Add tests that zero selected gravity magnitude returns zero stochastic gravity acceleration and zero Jacobian.
- [ ] Add a singular-RTN test where `norm(cross(r, v))` is too small and the helper errors with `EvalStochasticGravityRTNAccel:SingularRTNFrame`.
- [ ] Add finite-difference validation for the analytical `3x6` stochastic-gravity Jacobian, including nonzero velocity partials from the RTN frame.

## Task 6: SRP-Related Stochastic Residual In Illumination Frame

**Files:**
- Create: `matlab/simulation_models/accelerations/EvalStochasticSRPAccel.m`
- Create: `matlab/simulation_models/accelerations/EvalJac_StochasticSRPAccel.m`
- Modify: `matlab/simulation_models/dynamics/evalRHS_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- Test: `tests/matlab/simulation_models/accelerations/testStochasticSRPAccel.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityStochastic.m`

**Interfaces:**
- `EvalStochasticSRPAccel(dEvalTime, dPosSC_IN, dPosSun_IN, dAccSRP_IN, bIsSRPActive, strSRPChannel)` returns `dAccStochasticSRP_IN`.
- `EvalJac_StochasticSRPAccel(dPosSC_IN, dPosSun_IN, dAccSRP_IN, dJacSRP_IN, bIsSRPActive, dUnitSRPFrame)` returns a `3x6` orbit-state partial.
- The SRP frame is sun-line based so it can be used for both cannonball and panel SRP:
  - first axis is along the deterministic radiation-pressure line;
  - the two transverse axes are built deterministically from the sun-line and an inertial fallback axis;
  - velocity partial is zero unless a future SRP model explicitly depends on velocity.

- [ ] Add tests that inactive SRP, missing Sun ephemeris, and eclipse return zero stochastic SRP acceleration and zero Jacobian.
- [ ] Add tests that SRP stochastic acceleration scales with `norm(dAccSRP_IN)`.
- [ ] Add tests for frame construction when the sun-line is close to the first fallback axis.
- [ ] Add a zero Sun-spacecraft distance test that errors with `EvalStochasticSRPAccel:InvalidSunLine`.
- [ ] Add finite-difference validation for cannonball-SRP and panel-SRP scaled stochastic Jacobians using the selected deterministic SRP Jacobian.

## Task 7: RHS And Jacobian Integration

**Files:**
- Modify: `matlab/simulation_models/dynamics/evalRHS_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/private/ResolveInertialDynMaxFidelityConfig.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m`
- Test: `tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityStochastic.m`

**Interfaces:**
- `strDynParams.strStochasticAccelData.strGravityRTN` activates the gravity channel when `bIncludeStochasticAcceleration=true`.
- `strDynParams.strStochasticAccelData.strSRP` activates the SRP channel when `bIncludeStochasticAcceleration=true`.
- Legacy `strDynParams.strStochasticAccelData.dAccelGrid` remains a temporary absolute inertial fallback and should be reported in diagnostics as `charStochasticAccelMode='legacy_inertial_absolute'`.
- Diagnostics include:
  - `dAccSelectedGravity_IN`;
  - `dAccSelectedSRP_IN`;
  - `dAccStochasticGravity_IN`;
  - `dAccStochasticSRP_IN`;
  - `dAccStochastic_IN`;
  - selected gravity/SRP model IDs.

- [ ] Add RHS tests that enabling each channel changes only the acceleration rows.
- [ ] Add RHS tests that both channels add linearly after deterministic force selection.
- [ ] Add RHS tests that SRP stochastic contribution is zero during eclipse while gravity stochastic contribution remains active.
- [ ] Add Jacobian tests that RHS finite differences match `evalJac_InertialDynMaxFidelity(...)` when one or both stochastic channels are active.
- [ ] Add tests that legacy absolute inertial stochastic data still produces the existing behavior.

## Task 8: Documentation, Changelog, And Final Review

**Files:**
- Modify: `matlab/simulation_models/accelerations/GenerateGaussMarkovAccelSeq.m`
- Modify: `matlab/simulation_models/accelerations/EvalGaussMarkovAccel.m`
- Modify: `matlab/simulation_models/dynamics/evalRHS_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/evalJac_InertialDynMaxFidelity.m`
- Modify: `matlab/simulation_models/dynamics/private/ResolveInertialDynMaxFidelityConfig.m`
- Review: all new helper/test files from Tasks 3-7.

**Interfaces:**
- Public headers explain absolute legacy profiles versus dimensionless source-specific channels.
- Changelog entries name stochastic gravity RTN, stochastic SRP, and exclusive model selection separately.
- Final review reports staging candidates without committing.

- [ ] Run focused tests:
  - `runtests('tests/matlab/simulation_models/accelerations/testGaussMarkovAccelSeq')`
  - `runtests('tests/matlab/simulation_models/accelerations/testStochasticGravityRTNAccel')`
  - `runtests('tests/matlab/simulation_models/accelerations/testStochasticSRPAccel')`
  - `runtests('tests/matlab/simulation_models/accelerations/testComputeQuadsModelSRP')`
  - `runtests('tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity')`
  - `runtests('tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityPanelSRP')`
  - `runtests('tests/matlab/simulation_models/dynamics/testEvalJac_InertialDynMaxFidelityStochastic')`
- [ ] Run `git diff --check` and `git diff --cached --check`.
- [ ] Review commit split before staging:
  - current panel-SRP analytical Jacobian and tests;
  - deterministic model selection exclusivity and helper extraction;
  - stochastic channel data contract;
  - gravity RTN stochastic channel;
  - SRP stochastic channel;
  - RHS/Jacobian integration and documentation.

## Open Review Points Before Implementation

- [ ] Confirm whether explicit SRP model selection should use numeric IDs only, string labels only in diagnostics, or a new enum-like class. Numeric IDs are recommended for codegen and stable struct layout.
- [ ] Confirm whether legacy absolute inertial stochastic profiles should be removed immediately after source-specific channels land, or retained for one branch as compatibility.
- [ ] Confirm the SRP stochastic frame name and convention. Recommended first implementation is `SRP_SUNLINE`, not spacecraft-body frame, because it supports both cannonball and panel SRP without requiring attitude for cannonball cases.
- [ ] Confirm whether the default relative amplitude randomization should be independent per channel or shared across gravity and SRP for one truth realization. Recommended first implementation is independent seeds per channel.
