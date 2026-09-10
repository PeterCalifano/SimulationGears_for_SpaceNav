# Automatic-Differentiation Propagation Restoration Plan

## Goal

Restore SimulationGears-owned numerical and CasADi-compatible propagation
without reintroducing scheme-specific interval wrappers. Keep ordinary
SimulationGears setup independent of CasADi, and keep the reusable integration
steps below the interval-propagator layer.

## Approved Contracts

- [x] Retain one AutoDiff-specific adaptive interval boundary:
  `PropagateAdaptiveStepAutoDiff`.
- [x] Restore separately callable `PropagateRK4StepAutoDiff` and
  `PropagateRK8StepAutoDiff` integration steps under
  `matlab/simulation_models/propagators/integrators/autodiff/`.
- [x] Use explicit `strDynParams` and `strModelConfigFlags` inputs; do not add
  `varargin`, repeating argument blocks, or cell argument packs.
- [x] Keep function names within the 31-character code-generation limit.
- [x] Preserve the state-history-first and time-grid-second interval output
  order, one state per row, and exact endpoints.
- [x] Do not add a second AutoDiff fixed-step interval wrapper. Symbolic fixed
  steps are composed from the separately callable RK4/RK8 step functions.
- [x] Keep EstimationGears implementations and the standalone
  `/home/peterc/devDir/MathCore_for_SpaceNav` checkout unchanged.

## Stage 0 — Legacy Review And Test-First Baseline

- [x] Confirm the legacy `PropagateRKF45symCompatible` is fixed-step despite
  its name, supports only forward intervals, allocates inconsistent output
  layouts, and contains the erroneous coefficient `1408.8/2565.0`.
- [x] Confirm legacy `StepRK4_AutoDiff` repeats the same Fehlberg formula
  rather than implementing classical RK4 and has an invalid arguments block.
- [x] Confirm legacy `StepRK8_AutoDiff` is a two-input passthrough stub.
- [x] Confirm MATLAB R2024b can load the installed CasADi 3.6.7 distribution
  from `/home/peterc/devDir/casadi/matlab/casadi_matlab_3.6.7`.
- [x] Add focused failing numerical tests for the approved public names,
  explicit RHS inputs, double-kernel parity, MATLAB `ode45` parity, backward
  propagation, exact endpoints, and zero duration.
- [x] Add focused failing CasADi tests for SX RK4/RK8 flow construction, MX
  adaptive propagation, Jacobians, shortened final output intervals, backward
  propagation, and zero duration.

The R2024b red gate failed all four numerical tests and all three CasADi tests
with `MATLAB:UndefinedFunction` on the approved public names before the
implementation files were added.

## Stage 1 — Restored AutoDiff Providers

- [x] Implement numerical RK4/RK8 behavior by delegating double states to the
  canonical validated SimulationGears kernels.
- [x] Implement SX/MX RK4/RK8 step graphs with the same classical RK4 and
  thirteen-stage Fehlberg RK8 formulas.
- [x] Delegate double adaptive propagation to the canonical
  `PropagateAdaptiveStep` RKF45 implementation.
- [x] Use CasADi CVODES for symbolic MX interval propagation so adaptive
  accept/reject logic executes when the graph is evaluated rather than
  branching on unresolved symbolic errors in MATLAB.
- [x] Keep CasADi loading caller-controlled and absent from
  `SetupSimGears`.
- [x] Return an empty statistics struct for symbolic graph construction,
  because accepted/rejected internal steps exist only when the graph is
  evaluated.
- [x] Add complete sectioned documentation and purpose-oriented comments to
  every public callable.

## Stage 2 — MATLAB R2024b Verification And Review Gate

- [x] Run the numerical AutoDiff suite without adding CasADi to the MATLAB
  path.
- [x] Run the CasADi suite after explicitly adding the installed 3.6.7 path
  and assert the loaded version.
- [x] Run the existing numerical-propagator suite to exclude regressions in
  fixed/adaptive source behavior and MATLAB ODE parity.
- [x] Run Code Analyzer on all changed MATLAB source and test files.
- [x] Run `git diff --check`.
- [x] Stage only the AutoDiff implementation, focused tests, and this plan.
- [x] Inspect the complete cached diff and run
  `git diff --cached --check`.
- [x] Stop for the user commit before modifying the nested MathCore
  repository.

R2024b verification passed 20/20 tests without CasADi on the MATLAB path:
4/4 restored-interface numerical tests plus the existing 16/16 numerical
propagator suite. The explicitly loaded CasADi 3.6.7 suite passed 5/5 tests.
Maximum full-history absolute differences from tightly configured `ode45`
were `1.566527463304e-11` for the double RKF45 path,
`3.710993179418e-10` for forward MX CVODES propagation, and
`1.435924157356e-10` for backward MX propagation. Returned endpoints were
exactly `2.0`, `1.3`, and `0.0`, respectively. Code Analyzer reported zero
messages in all three implementation files and both focused test files.

## Final Reconciliation

- [x] Commit the restored AutoDiff providers and focused tests at `e7820d2`.
- [x] Verify that the RK4 and RK8 SX graphs can be wrapped in
  `casadi.Function` and emitted through `Function.generate`. Generated C
  contained constant coefficient literals and no arithmetic division
  instructions from the Runge-Kutta tableau.
- [x] Keep this as code-generation compatibility rather than a new public
  builder/API. No standalone-C claim is made for the adaptive CVODES path.
- [x] Retire the superseded MathCore AutoDiff files, migrated numerical
  propagators, experimental `PropagateRKF45v2`, and ten deprecated integrator
  files at nested MathCore commit `06337d2`.
- [x] Advance the SimulationGears MathCore gitlink to that clean revision at
  parent commit `b01fc50`.

Fresh MATLAB R2024b reconciliation on 28-07-2026 passed the 20/20 generic and
numerical AutoDiff tests plus the explicitly loaded CasADi 3.6.7 suite at 5/5.
