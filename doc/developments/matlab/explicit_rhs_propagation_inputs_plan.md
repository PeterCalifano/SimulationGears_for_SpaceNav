# Explicit RHS Propagation Inputs Plan

## Goal

Expose one fixed-step and one adaptive interval propagator that pass the
standard SimulationGears dynamics payload and model configuration explicitly
to every RHS evaluation. Keep the numerical provider generic enough for
ordinary mathematical systems and the max-fidelity inertial dynamics, without
adding a specialized interval-propagation API.

## Approved Public Contracts

- [x] Use the explicit fixed-step contract:
  `[dxStateHistory, dTimeGrid, strPropagationStats] = PropagateFixedStep(fcnStateDerivative, dTimeSpan, dxInitialState, dMaximumStep, strDynParams, strModelConfigFlags, enumFixedStepScheme)`.
- [x] Use the explicit adaptive contract:
  `[dxStateHistory, dTimeGrid, strPropagationStats] = PropagateAdaptiveStep(fcnStateDerivative, dTimeSpan, dxInitialState, dInitialStep, strDynParams, strModelConfigFlags, Name=Value)`.
- [x] Require the RHS contract
  `dxStateDerivative = fcnStateDerivative(dTime, dxState, strDynParams, strModelConfigFlags)`.
- [x] Pass `struct()` explicitly for parameter-free mathematical systems.
- [x] Do not add `varargin`, repeating argument blocks, cell argument packs,
  scheme-specific interval aliases, or a specialized max-fidelity interval
  provider.
- [x] Keep `enumFixedStepScheme` and `strModelConfigFlags` compile-time
  constant for generated targets while keeping `strDynParams` as runtime
  input.
- [x] Preserve state-history-first output, one state per row, a column time
  grid, exact interval endpoints, and optional propagation statistics.

## Stage 0 — Baseline And Test-First Contract

- [x] Start from clean branch `feature/implement-ode-propagators` at
  `472efb8`.
- [x] Add behavioral tests proving both explicit structs reach every fixed and
  adaptive RHS evaluation.
- [x] Add direct single-step tests for `PropagateRK2HeunStep`,
  `PropagateRK4Step`, and `PropagateRK8Step` with the same explicit structs.
- [x] Update parameter-free convergence and MATLAB ODE parity tests to pass
  explicit empty structs and retain independently derived tolerances.
- [x] Preserve timestamp tests for forward, backward, shortened-final-step,
  zero-duration, requested-grid, and adaptive propagation.
- [x] Add a code-generation regression that binds
  `evalRHS_InertialDynMaxFidelity`, runtime `strDynParams`, constant
  `strModelConfigFlags`, and constant `enumFixedStepScheme`.
- [x] Run the focused tests before implementation. MATLAB execution failed
  with `MATLAB:TooManyInputs` at `PropagateRK2HeunStep`, and generated code
  reached the shared provider before failing with “Too many inputs to function
  `PropagateFixedStep`”.
- [x] Record the R2024b entry-point constraint: `coder.Constant` cannot contain
  a function handle (`Coder:common:TypeSpecConstantTypeValue`). Therefore the
  codegen regression uses the thin `CodegenInertialFixedStepProbe` RHS-binding
  entry point with no integration algorithm.

## Stage 1 — Shared Provider Implementation

- [x] Add `strDynParams` and `strModelConfigFlags` as explicit arguments to
  `PropagateFixedStep`, `PropagateAdaptiveStep`, and all three fixed-step
  integration functions.
- [x] Forward both structs unchanged at every RHS evaluation.
- [x] Keep the scheme selection outside the fixed-step loop and retain
  `coder.mustBeConst(enumFixedStepScheme)`.
- [x] Update `CGeneralPropagator` to bind its legacy caller-specific parameter
  list into the explicit four-input RHS contract and pass explicit empty
  structs to the shared provider.
- [x] Keep the ordinary MATLAB setup independent of CasADi and MATLAB Coder.
- [x] Update complete sectioned documentation and purpose-oriented comments
  for every modified public MATLAB callable.

## Stage 2 — R2024b Verification And Review Gate

- [x] Load the repository with `matlab/SetupSimGears.m` and print `which` for
  the shared providers and max-fidelity RHS before testing.
- [x] Run the focused numerical-propagator suite under MATLAB R2024b: 16/16
  tests passed.
- [x] Run numerical parity against `ode45`, `ode78`, and `ode113` with
  reasonable method-specific tolerances and exact timestamp assertions.
- [x] Build and run the max-fidelity fixed-step MEX probe. The shared provider
  and real RHS compiled, source/MEX histories matched within `1.0e-13`, and a
  post-build GM change proved `strDynParams` remains runtime data.
- [x] Inspect generated code for constant-scheme branch pruning and for a
  runtime max-fidelity payload in the generated entry-point contract. The core
  signature retains `strDynParams` but omits flags and scheme, the loop has
  exactly four max-fidelity RHS calls and no scheme switch or RK8 stages, and
  the configuration resolver/model-selection tokens remain only in generated
  comments.
- [ ] Run the broader SimulationGears MATLAB regression suite. The first full
  attempt exposed established environment/test failures including missing
  MICE, missing setup helpers and assets, stale dataset constructors, an
  unimplemented test, and the existing J2 scaling assertion. Focused
  source-owner suites remain the commit gate.
- [x] Run the focused source-owner gate: 20/20 propagator tests, 10/10
  max-fidelity RHS tests, and 2/2 max-fidelity codegen/parity tests passed.
  The separate MEX-output hygiene suite passed 6/6.
- [x] Run Code Analyzer on every modified source and test file. The changed
  scope is clean; `CGeneralPropagator` retains the same 15 warnings as `HEAD`
  in untouched legacy stubs.
- [x] Run `git diff --check`, stage only this implementation, its focused
  tests, and this directly corresponding plan, then inspect the complete
  cached diff and `git diff --cached --check`.
- [x] Stop for the user commit before changing or staging the COSMICA
  consumer.

## Downstream COSMICA Gate

- [x] Compile the shared `PropagateFixedStep` implementation through the
  builder-private max-fidelity RHS-binding adapter with runtime `strDynParams`,
  constant truth/model flags, and constant fixed-step scheme. COSMICA commit
  `75b0ff3` owns the consumer and artifact workflow.
- [x] If COSMICA still requires a named MEX entry point for artifact
  management, keep it builder-private and limited to binding the shared RHS
  plus adapting output orientation; it must not contain an integration
  algorithm or remain a public COSMICA propagation API. The retained
  `CosmicaTruthMexAdapter` satisfies this seam, while generated callables use
  readable `PropagateTruth_<scheme>_<gravtype>` names.
- [x] Delete `PropagateCosmicaTruthFixedStep`, `propagate_env_src`, and
  `rhs_dynamics_env` after behavioral consumer parity confirms no runtime
  caller remains.
- [x] Re-run point-mass RK4, polyhedron RK4, and registry degree-16 RK8
  source/MEX parity plus the one-day timestamp, endpoint, and performance
  benchmark.

Final reconciliation on 28-07-2026 passed 18/18 focused max-fidelity RHS,
generated-code parity, and MEX-output hygiene tests. Fresh COSMICA R2024b
verification passed the fixed-step source/MEX harness, registry-backed
degree-16 RK8 parity, object-level MATLAB/MEX dispatch, interface/default
contracts, and full Monte Carlo execution/preflight. The previously recorded
manual one-day diagnostic returned exactly 86,401 timestamps and the requested
endpoint; warmed medians were 279.141226 seconds for MATLAB and 11.575337
seconds for MEX, with maximum full-history differences of
`7.771561e-15 km` and `3.862470e-19 km/s`. Timing remains a manual
optimization diagnostic rather than an automatic acceptance threshold.
