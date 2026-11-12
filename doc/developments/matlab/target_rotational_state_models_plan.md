# Target Rotational-State Models

## Release coordination: 5 October 2026

The MATLAB release refurbishment plan of the COSMICA repository
(`doc/developments/matlab_major_release_refurbishment_plan.md`)
coordinates current model-consolidation fixes with this destination's rotational
history and perturbation implementation. Preserve the approved physical chart
and boundary conventions below. Stages 0/1 are authorized now; fresh numerical,
consumer, and simulation acceptance remain separate gates, not historical passes.

Fresh Stage 0/1 baseline is qualified: 37 focused MATLAB rotational, covariance,
factory/interpolation and environment tests passed, clean exit 0. Latest maintained
model-consolidation input is `459b8cc5`; physical positive rate is reconciled with
direct MATLAB consumers. HEAD/index/gitlinks are unchanged. Live SPICE/deployed
MEX and full-core acceptance remain later gates. Pause for review/consolidation.

This plan closes the generic target rotational-state capability in SimulationGears. COSMICA owns scenario
orchestration, seed derivation and stream ordering, run configuration, concrete condition materialization, and
campaign provenance. Subject to explicit approval of the exact final split, SimulationGears also owns reusable
covariance sampling and its generic distribution enum. MathCore owns reusable quaternion, Lie-group, and SPD numerical
primitives. SimulationGears owns the physical rigid-body model, deterministic perturbation map, history integration,
runtime payload, and any MATLAB/C++ bridge. Nav-Frontend is out of scope; any cleanup in Nav-Backend is a separate
repository-owned change.

## Approved Design Direction

- [x] Keep the existing MathCore passive-quaternion/right-increment public contract unchanged. Correct internal
  numerical defects and optimize the implementation only under invariant, convergence, and consumer tests.
- [x] Preserve the existing MathCore rigid-body and quaternion update behavior, with no frame or multiplication-side
  flag. Its Euler state is body-frame angular velocity and its passive quaternion maps to `C_TBfromIN`.
- [x] Keep the public SimulationGears contract as `R_INfromTB` and
  own the conversion at this boundary: integrate `C_TBfromIN = transpose(R_INfromTB)`, then transpose the output back.
- [x] Make a stateless whole-history interface the canonical SimulationGears seam for MATLAB, generated MEX, and native
  C++ implementations. Do not preserve stateful or per-step wrapper crossings solely for compatibility with the draft.
- [x] Replace inertia rejection/resampling and principal-eigenaxis perturbations with a rejection-free six-coordinate
  physical chart based on the centered second-moment matrix.
- [x] Treat target spin axis and its provenance as one optional pair. Never synthesize `DEFAULT_PLUS_Z` in the canonical
  runtime payload.
- [x] Express angular-velocity capability with an always-present typed `EnumTargetAngularVelocityRegime`, not field or
  property presence, an empty value, or a parallel boolean.

## Stage 1: Source And Configuration Contract

- [x] Validate the typed `SPICE | DATASET | GENERATED` source and
  `SOURCE_PROVIDED | KINEMATIC_CONST_INERTIAL_RATE | RIGID_BODY_TORQUE_FREE` model combinations in MATLAB.
- [x] Verify strict public configuration parsing, complete generated-state serialization, and unknown-field rejection.
- [x] Verify generated target rotation skips only the source attitude/rate query while retaining Sun, third-body,
  gravity, shape, and spacecraft dynamics data.
- [x] Reject source-inapplicable integration and perturbation fields instead of retaining ignored configuration.
- [x] Keep torque inputs limited to the smallest justified contract for this lane: zero torque for torque-free truth and,
  only if an existing caller needs it, a constant body-frame torque. A generic callback hierarchy requires a separate
  approved use case.

## Stage 2: Quaternion Convention And Rigid-Body Correctness

- [x] Use an isolated MathCore worktree for the approved internal RKMK4 correction. Preserve its public convention and
  do not edit the canonical checkout, a nested dependency gitlink, or mirrored copies.
- [x] Add independent non-identity spherical-body and noncommuting-rate oracles before implementation changes. Do not
  use a reference ODE that repeats the same quaternion multiplication convention as the code under test.
- [x] At the SimulationGears torque-free boundary, convert the declared initial quaternion through
  `R_INfromTB -> C_TBfromIN -> q_TBfromIN`, call the unchanged rigid-body integrator, and return
  `R_INfromTB = transpose(C_TBfromIN)` with `omega_IN = R_INfromTB * omega_TB`.
- [x] Correct the constant-inertial-rate analytic model to the same declared direction:
  `R(t) = Exp(+omega_IN * dt) * R(0)`. The previous negative sign belongs to the inverse passive matrix.
- [x] Apply the same active-matrix sign at the source and uncertainty boundaries: retain the angular velocity returned
  by `cspice_xf2rav(cspice_sxform(IN, TB))` without negation, and propagate inertial left-error covariance with
  `delta_theta_dot = skew(omega_IN) * delta_theta + delta_omega_IN`.
- [x] Preserve the generic MathCore convention and add no body-rate selection API. Correct the RKMK4
  right-trivialized `dexp^-1` bracket sign so the existing right group update recovers fourth-order noncommuting-rate
  behavior. Consider removal of the currently unused injected quaternion-integrator property only as a separately
  tested simplification after serialized-object and caller audit.
- [x] Verify quaternion norm/sign continuity, non-identity spherical equivalence, torque-free energy, inertial angular
  momentum `R_INfromTB * J_TB * omega_TB` as a vector, and agreement across Euler/RK4/RKMK within method-appropriate
  tolerances.
- [x] Check the torque-free history against closed-form solutions that share no code with the integrator: the
  axisymmetric body (body rate rotating about the symmetry axis, attitude precessing about the inertial angular
  momentum) and the triaxial body rates from the Jacobi elliptic functions, both over at least three periods.
- [x] Audit all callers again after the adapter and stop for review if a previously undiscovered consumer interprets the
  draft generated histories with the inverse attitude direction.

## Stage 3: Whole-History Integration And Performance

- [x] Refactor the corrected MATLAB implementation into one stateless, codegen-safe whole-history kernel owned by
  SimulationGears. Retain a small class adapter only if a real stateful caller requires it.
- [x] Add a MATLAB Coder MEX target using the existing SimulationGears builder/output conventions and record it as the
  first optimized baseline.
- [x] Select generated execution through the typed `EnumTargetRotationalStateBackend`. Keep MATLAB as the artifact-free
  default; a MEX request builds or reuses an exact source/ABI-fingerprinted profile and never silently falls back after
  a build, resolution, or invocation failure.
- [x] Evaluate whether a handwritten C++20 batch integrator is justified after generated-MEX measurement. The generated
  whole-history target is already 30.2 times faster than the legacy class on the representative ten-day approach grid,
  so defer a second native implementation unless an end-to-end performance target remains unmet.
- [x] Add one thin MATLAB wrapper call for an entire requested history, with validated typed inputs and preallocated
  attitude/body-rate outputs. Do not cross the language boundary once per integration step and do not involve GTSAM,
  ROS 2, COSMICA, or navigation repositories.
- [x] Establish MATLAB-reference and generated-MEX parity over spherical, asymmetric torque-free, nonuniform-grid,
  two-output-epoch, and invalid-input fixtures. Native-C++ parity remains conditional on a future C++ implementation.
- [x] Benchmark representative approach durations and sample cadences, recording absolute runtime, speedup, compiler,
  build type, and numerical tolerances. Retain handwritten C++ only if it materially improves the generated-MEX path
  enough to justify a second implementation.
- [x] Select one maintained fast runtime backend plus one clear reference default. Remove redundant adapters,
  duplicated validation, and unused integrator state after parity and performance acceptance.

## Stage 4: Deterministic Rotational-State Perturbation

- [x] Expose one deterministic operation that applies a supplied initial-attitude tangent error, a supplied
  dynamics-model-frame angular-velocity error, and a supplied inertia-chart error. Keep its frame conventions explicit
  and keep RNG, distribution, seed, campaign, and retry policy outside SimulationGears.

- [x] Replace the draft principal-log-moment/principal-axis schema with a six-coordinate chart of
  `Sigma = 0.5 * trace(J) * I - J`, where `J` is inertia about the center of mass.
- [x] Map each nominal `Sigma` bijectively through a relative lower-triangular factor with logarithmic diagonal entries
  and direct off-diagonal entries. Compose that factor with the nominal Cholesky factor, then reconstruct
  `J = trace(Sigma) * I - Sigma`; zero relative coordinates reproduce the nominal inertia exactly.
- [x] Follow the corrected Cholesky factor ordering, not the inconsistent equation in the original 2022 publication;
  make chart-to-matrix and matrix-to-chart round-trip tests the executable convention.
- [x] Guarantee by construction that every finite chart vector produces symmetric positive-definite `Sigma`, and hence
  positive principal moments satisfying strict triangle inequalities. Reject malformed/nonfinite configuration, not
  valid random draws.
- [x] Keep the deterministic chart application and physical validation in SimulationGears with no RNG, distribution,
  seed, or retry policy. Add a MathCore SPD helper only if a second real consumer justifies it.
- [x] Replace `PerturbInertiaTensor.m` in place, correct its public function name to match the file, and give it the new
  deterministic nominal-inertia plus six-coordinate chart-error contract. Remove its global `randn`, additive-moment,
  and rejection-loop implementation rather than adding a competing public function.
- [x] Document that configured Gaussian or uniform-ellipsoid covariance is covariance of chart coordinates. The
  nonlinear physical map does not promise that the arithmetic mean/covariance of inertia entries equals the nominal
  tensor or the chart covariance.
- [x] Reserve extension to mass/center-of-mass perturbation for a full 4-by-4 pseudo-inertia chart rather than adding an
  unrelated second parameterization.

## Stage 5: Runtime History And Capability Payload

- [x] Add `EnumTargetAngularVelocityRegime` with `UNAVAILABLE`, `CONSTANT_INERTIAL`, and `TIME_VARYING`, and carry one
  scalar value on every canonical rotational history and fitted runtime attitude payload.
- [x] Classify generated and source histories only after validating their complete angular-velocity sequence. Require
  complete aligned rate histories for both non-`UNAVAILABLE` enum values.
- [x] Remove `dNominalAngVel_IN` from canonical inputs and outputs. In the `CONSTANT_INERTIAL` branch, obtain the rate
  from the validated `dAngVel_IN` history; never infer capability from the first sample.
- [x] Make consumers switch on the typed enum. Do not use `isfield`, `isprop`, empty values, or a boolean to discover
  constant-rate capability.
- [x] Verify torque-free histories through the corrected integrator with no false constant-rate enum and no constant
  spin-axis metadata.
- [x] Remove the canonical `DEFAULT_PLUS_Z` fallback from `EphemeridesDataFactory`. Preserve axis metadata only as a
  complete, type-valid axis/provenance pair supplied by the source or generated constant-rate model.
- [x] Omit spin-axis metadata for zero-rate histories unless a source explicitly defines and validates it; zero angular
  velocity alone does not determine an axis.
- [x] Emit one canonical Chebyshev-compatible attitude payload for COSMICA without duplicating truth state in an
  algorithm-config or navigation-owned structure.
- [x] Keep propagated target-attitude covariance available only for `CONSTANT_INERTIAL`; torque-free or source
  histories classified otherwise require `CONSTANT_INITIAL` until a history-driven variational covariance model is
  designed.

## Stage 5A: Generic Sampling Boundary

- [x] Obtain explicit approval for the exact ownership split before changing the current COSMICA sampling interface.
- [x] Add one stateless SimulationGears covariance sampler accepting a caller-owned `RandStream`, a finite symmetric
  positive-semidefinite covariance, sample count, and typed `GAUSSIAN | UNIFORM_SOLID_ELLIPSOID` distribution.
- [x] Keep covariance validation/factorization and covariance-matched draws in SimulationGears, with no seed
  derivation, global RNG mutation, campaign ordering, retries, YAML, materialization, or provenance.
- [x] Replace the COSMICA-local distribution enum with the SimulationGears enum and reduce
  `SampleTargetRotationalStateConfig` to campaign stream orchestration plus calls to deterministic perturbation
  application. Do not introduce a nested one-consumer configuration class.
- [x] Move generic distribution/covariance tests to SimulationGears; retain COSMICA tests for stable child seeds,
  stream independence, configuration wiring, truth materialization, and provenance.

## Stage 6: Validation, Simplification, And Pause Gates

- [x] Run focused MathCore convention tests, SimulationGears physical-chart tests, MATLAB/MEX parity, environment
  builder, and ephemerides-factory tests.
- [x] Run the relevant broader SimulationGears suites through the integrated dependency revision without touching the
  active campaign checkout or advancing a gitlink during this worktree phase.
- [x] Run the COSMICA config/MC contracts and frontend-off approach smoke only through the real sibling path setup; do
  not edit Nav-Frontend or Nav-Backend to make these gates pass.
- [x] Review complete repository diffs for scope, public documentation, duplicated policy, unnecessary classes/helpers,
  and behavior-preserving simplification before staging or handoff.
- [ ] Pause for user direction after repeated unrelated failures, a newly discovered production dependency on the old
  convention, an ownership conflict, or a change that materially alters the approved chart, wrapper, or payload
  interfaces.
- [ ] Pause at the external Nav-Frontend prerequisite. Event rendering/observation integration remains deferred, while
  the generic history and observation boundaries must not preclude a future timestamped event model.

## Research Basis

The physical map follows the fully physical inertia conditions in Wensing, Kim, and Slotine (2017) and Traversaro et
al. (2016). The global smooth chart is based on Rucker and Wensing (2022), with the factor-ordering correction published
in 2026. Quaternion frame/trivialization terminology follows Sola (2017), and the RKMK checks follow Munthe-Kaas
(1998).

- Wensing, Kim, and Slotine, *Linear Matrix Inequalities for Physically-Consistent Inertial Parameter Identification*:
  <https://arxiv.org/abs/1701.04395>
- Traversaro et al., *Identification of Fully Physical Consistent Inertial Parameters using Optimization on
  Manifolds*: <https://arxiv.org/abs/1610.08703>
- Rucker and Wensing, *Smooth Parameterization of Rigid-Body Inertia*:
  <https://par.nsf.gov/servlets/purl/10347458>
- Rucker and Wensing, correction to the Cholesky-factor equation: <https://doi.org/10.1109/LRA.2025.3646288>
- Sola, *Quaternion Kinematics for the Error-State Kalman Filter*: <https://arxiv.org/abs/1711.02508>
- Munthe-Kaas, *Runge-Kutta Methods on Lie Groups*: <https://doi.org/10.1007/BF02510919>

Implementation status (24-08-2026): the typed source/configuration contract, rejection-free relative inertia chart,
deterministic perturbation operation, optional spin-axis pair, and angular-velocity capability enum are implemented in
the isolated SimulationGears worktree. `EphemeridesDataFactory` no longer fabricates a spin axis and clears stale
rate/axis context before installing one validated typed history; legacy dataset constructors may still provide their
own explicit source-level default. The generated constant-rate model, SPICE adapter, torque-free transpose adapter,
and constant-rate covariance propagator now use one positive-sign active `R_INfromTB` convention.

A separate MathCore worktree corrects the right-trivialized RKMK4 differential sign and preserves nonzero rotations
below the former exponential-map cutoff. Its rigid-body and quaternion suites pass 16/16 and 17/17. With that exact
MathCore worktree first on the MATLAB path, SimulationGears passes rotational history 14/14, generated-MEX parity 1/1,
physical inertia 4/4, target-attitude covariance 8/8, environment construction 6/6, and ephemerides factory 6/6.
COSMICA's approach rotational-state seam and MC generation contracts also pass with this isolated dependency. The
dependency gitlink remains unchanged.

The broader SimulationGears regression pass additionally covers target-attitude covariance 8/8, environment
construction 6/6, and ephemerides factory 6/6. COSMICA run-config 14/14, Monte Carlo set configuration 5/5, approach
trajectory Tests 0-16, full-core and approach-only bounded smokes, and the opt-in frontend-disabled propagated
photometry smoke pass with the isolated worktrees selected explicitly. Sparse-worktree shape data is supplied through
the supported `SIMGEARS_DATA_ROOT` override; no campaign checkout or generated artifact is modified.

On a ten-day grid with 432,000 internal steps and 14,401 outputs, the corrected legacy class took 5.611 s, stateless
MATLAB took 2.867 s, and MEX took 0.186 s: 30.2 times faster than legacy, with `6.25e-14` maximum sign-invariant
quaternion error and zero angular-rate error against the source. This satisfies the current performance objective
without a second handwritten C++ implementation. The generic sampler now vectorizes arbitrary-dimension Gaussian and
uniform-solid-ellipsoid draws from a caller-owned `RandStream`; zero samples validate without advancing the stream.
Its five focused tests cover deterministic Gaussian parity, global-RNG isolation, empirical uniform covariance/radius,
singular PSD validation, enum configuration compatibility, and malformed covariance rejection. A separate isolated
Nav-Backend worktree now consumes the typed complete history without `dNominalAngVel_IN`, rejects unsupported rate
regimes for the analytic covariance model, and rebuilds navigation-only active attitude with the positive sign while
keeping optional body-rate/spin-axis context coherent. Focused prior 9/9, perturbation 7/7, and initialization 18/18
tests pass; this SimulationGears worktree does not absorb that repository-owned code. Further MATLAB gates may run
individually; a process stalled after the known native wrapper teardown may be terminated after assertion results are
captured.

The generated backend is now an explicit numerical policy rather than path-presence behavior. The builder publishes
one complete profile under `matlab/mex/simulation_models/attitude` by default, or under a caller-supplied root for
testing and packaging. Profile identity covers the resolved source closure, builder source, MATLAB release, platform
MEX extension, and the fixed gateway contract; runtime grids and physical values remain variable inputs. MATLAB mode
does not resolve or create artifact storage. MEX selection is accepted only for torque-free integration, so analytic or
source-provided models cannot silently ignore an explicit generated-backend request.
