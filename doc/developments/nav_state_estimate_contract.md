# Generic Navigation-State Estimate Contract

## Purpose and ownership

`SNavStateEstimate` is the SimulationGears-owned value carrier for one
navigation state and its uncertainty at the same epoch. It composes an
`SNavState`; it does not inherit from `SNavState` and does not duplicate the
state timestamp, position, velocity, or attitude.

The class performs no propagation, interpolation, retiming, or delayed-state
correction. A filter or smoother adapter that holds a delayed estimate must
propagate the state and uncertainty before constructing a new estimate at a
different timestamp. Calling an estimate conversion or frame-change method
must not substitute or advance the composed state's timestamp.

The integration tranche removes legacy uncertainty, time-update policy, and
graph-result fields from `SNavState`. It is now the pure timestamped kinematic
value containing position, velocity, and attitude only.

## Public interface

```matlab
objEstimate = SNavStateEstimate(objNavState, dUncertaintyMatrix, ...
    enumUncertaintyRepresentation, enumStateLayout, enumAttitudeErrorConvention);
objStrictEstimate = SNavStateEstimate(objNavState, dUncertaintyMatrix, ...
    enumUncertaintyRepresentation, enumStateLayout, ...
    enumAttitudeErrorConvention, true);

dCovariance = objEstimate.getCovariance();
dInformation = objEstimate.getInformation();
objTransformedEstimate = objEstimate.changeReferenceFrame(dDCM_NewFrameFromFrame);
objRelativeState = objState.composeRightSide(objReferenceState);
```

The five value inputs are stored with public getters and private setters. The
optional `bStrictValidation` construction policy defaults to `false` and is not
stored. Every estimate carries an uncertainty matrix; an empty or sentinel
uncertainty is structurally invalid. State-only consumers continue to use
`SNavState`.

## Same-epoch relative state composition

`SNavState.composeRightSide` extends the existing `SPose3` relative-pose
contract to velocity while retaining the common state epoch. The operands must
have timestamps agreeing within an absolute `1.0e-6` seconds. The result keeps
the left operand's timestamp and is computed as

```text
t_relative = t_self
r_relative = r_self - r_other
v_relative = v_self - v_other
R_relative = R_other' * R_self
```

Position and velocity differences remain expressed in their shared reference
frame, matching `SPose3` translation semantics. The method returns a new value
and does not mutate either operand. States outside the timestamp tolerance are
rejected rather than implicitly propagated or retimed.

## Cross-repository ownership

- SimulationGears owns `SNavState` and `SNavStateEstimate`; neither type owns
  filter propagation policy or optimizer-native objects.
- Nav-Backend owns `SGraphStateEstimate`, including GTSAM pose/rotation values,
  individual marginals, and the complete graph-tangent joint covariance. Its
  `toNavStateEstimate` method extracts the generic position/velocity marginal.
- The recursive-filter bus remains the canonical filter-uncertainty source.
  Nav-System constructs a local `SNavStateEstimate` only where dispersion or
  another uncertainty-aware interface requires it.
- A smoother-to-filter reset associates an estimate at its unchanged epoch:
  either with the current filter state or with a timestamp-matched historical
  state retained by a sliding-window filter. If a consumer instead requires an
  estimate at a different epoch, its owning filter or smoother adapter must
  propagate the state and uncertainty before constructing that new estimate;
  no value class retimes it implicitly.

## Explicit metadata

`EnumStateUncertaintyRepresentation` has these values:

- `COVARIANCE`
- `SQRT_COVARIANCE_UPPER`
- `SQRT_COVARIANCE_LOWER`
- `INFORMATION`
- `SQRT_INFORMATION_UPPER`
- `SQRT_INFORMATION_LOWER`

`EnumNavStateUncertaintyLayout` has these values:

- `POSITION_VELOCITY`, with error ordering `[position; velocity]` and a
  6-by-6 uncertainty matrix.
- `POSITION_VELOCITY_ATTITUDE_ERROR`, with error ordering
  `[position; velocity; attitude error]` and a 9-by-9 uncertainty matrix.

`EnumAttitudeErrorConvention` has these values:

- `NONE`, required by `POSITION_VELOCITY`.
- `LEFT_FRAME`, valid only for `POSITION_VELOCITY_ATTITUDE_ERROR`.
- `RIGHT_POSE`, valid only for `POSITION_VELOCITY_ATTITUDE_ERROR`.

The representation and attitude convention are declared metadata. They must
never be inferred from symmetry, triangularity, dimension, magnitude, or any
other matrix contents.

## Matrix conventions and validity

For covariance representations, `M = P`. For information representations,
`M = I = P^-1`. Square-root factors use these exact conventions:

```text
upper factor R: M = R' * R
lower factor L: M = L  * L'
```

The constructor always requires the exact 6-by-6 or 9-by-9 shape selected by
the layout and a compatible attitude-error convention. With
`bStrictValidation=true`, direct covariance and information matrices must also
be finite, symmetric, and positive definite; square-root factors must be
finite, have the declared exact triangular orientation, and have a strictly
positive diagonal. The default trusted path skips these matrix scans and
factorizations so repeated filter/smoother construction does not pay for
validation already owned by the estimator.

Covariance/information conversion uses Cholesky factorization and triangular
linear solves. It must not form a matrix inverse with `inv`. A getter returns a
derived matrix without changing the stored matrix or representation enum.

Strict construction reports numerical validation failures through stable
identifiers; structural layout/convention failures are reported in both modes:

- `SNavStateEstimate:InvalidAttitudeErrorConvention`
- `SNavStateEstimate:InvalidMatrixSize`
- `SNavStateEstimate:NonFiniteUncertaintyMatrix`
- `SNavStateEstimate:AsymmetricUncertaintyMatrix`
- `SNavStateEstimate:UncertaintyMatrixNotPositiveDefinite`
- `SNavStateEstimate:UncertaintyFactorNotUpperTriangular`
- `SNavStateEstimate:UncertaintyFactorNotLowerTriangular`
- `SNavStateEstimate:UncertaintyFactorNonPositiveDiagonal`

## Reference-frame transformation

Let `R` be `dDCM_NewFrameFromFrame`. The composed `SNavState` mean is changed
through its existing state-only `changeReferenceFrame` operation. The
uncertainty Jacobian is

```text
POSITION_VELOCITY:                         J = blkdiag(R, R)
POSITION_VELOCITY_ATTITUDE_ERROR/LEFT:     J = blkdiag(R, R, R)
POSITION_VELOCITY_ATTITUDE_ERROR/RIGHT:    J = blkdiag(R, R, I3)
```

Covariance transforms as

```text
P_new = J * P * J'
```

and information transforms as

```text
I_new = J'^(-1) * I * J^(-1).
```

The information expression is evaluated with left and right linear solves.
After transformation, the uncertainty is refactored into the estimate's
original representation; the representation, layout, convention, and
timestamp are unchanged.

For generated code, the constructor first validates the matrix dimension
against the explicit layout. The resulting private-set dimension may then
anchor fixed-size Jacobian allocation; this is specialization after validation,
not inference of layout or convention from matrix contents.

## Verification commands

Run from the authoritative SimulationGears checkout:

```bash
matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); run('matlab/SetupSimGears.m'); addpath(genpath('tests/matlab')); results = runtests({'tests/matlab/general_utils/datastructs/testSNavState.m','tests/matlab/general_utils/datastructs/testSNavStateEstimate.m'}); assertSuccess(results);"

matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); run('matlab/SetupSimGears.m'); addpath(genpath('tests/matlab')); results = runtests('tests/matlab/general_utils/datastructs', 'IncludeSubfolders', true); assert(all([results.Passed]));"

matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); run('matlab/SetupSimGears.m'); addpath(genpath('tests/matlab')); results = runtests('tests/matlab', 'IncludeSubfolders', true); assert(all([results.Passed]));"

matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); run('matlab/SetupSimGears.m'); files = {'matlab/general_utils/datastructs/SNavState.m','matlab/general_utils/datastructs/SNavStateEstimate.m','matlab/general_utils/datastructs/EnumStateUncertaintyRepresentation.m','matlab/general_utils/datastructs/EnumNavStateUncertaintyLayout.m','matlab/general_utils/datastructs/EnumAttitudeErrorConvention.m','tests/matlab/general_utils/datastructs/testSNavState.m','tests/matlab/general_utils/datastructs/testSNavStateEstimate.m'}; assert(all(cellfun(@(file) isempty(checkcode(file, '-id')), files)));"

git diff --check
```

The code-generation probe is disposable and must generate only under a
temporary directory outside the checkout. Its exact command and MATLAB versus
generated-result comparison are recorded with the implementation verification
evidence rather than as a permanent product test.

## Verification evidence — 11 August 2026

- The focused SimulationGears state/estimate suite passed 24 of 24 tests. It
  covers same-epoch relative-state composition, rejection beyond the
  `1.0e-6`-second timestamp tolerance, trusted construction without numerical
  factorization, and opt-in strict numerical rejection.
- Disposable MEX generation covered representative 6-state and 9-state
  covariance conversion and non-identity frame transforms. The six maximum
  MATLAB/MEX absolute differences were `2.22e-16`, `1.24e-11`, `0`, `0`,
  `7.28e-12`, and `0`, all below `1e-10`; the temporary probe was removed.
- A disposable primitive-input Coder probe for `SNavState.composeRightSide`
  produced exact MATLAB/MEX parity and preserved the
  `SNavState:CompositionTimestampMismatch` rejection identifier. The probe and
  generated artifacts remain outside the repository under
  `/tmp/simgears-nav-compose-codegen.fiUAJc`.
- Disposable trusted/strict constructor probes both generated successfully
  under `/tmp/simgears-nav-estimate-validation.7PeNv1`. The trusted MEX retained
  a structurally valid singular covariance exactly and its generated source
  contained no Cholesky or positive-definite validation path. The strict MEX
  retained an SPD covariance exactly and rejected the singular case with
  `SNavStateEstimate:UncertaintyMatrixNotPositiveDefinite`.
- Scoped Code Analyzer checks reported zero findings in all seven changed
  MATLAB source and test files.
- The broader SimulationGears suite was attempted. The new state/estimate tests
  remained green, while independent environment or legacy failures remained
  around missing MICE/setup dependencies, a legacy dataset constructor,
  explicit unimplemented tests, and existing CR3BP cases. The bounded run was
  stopped when `testRHS_CR3BP` ceased making progress.
- The Nav-Backend focused graph/adapter/reset/backend-step tests passed 55 of 55.
  The broader focused backend matrix completed with 96 passed, zero failed,
  and two filtered tests due to an unavailable range-prior factor and missing
  loop-closure dataset.
- Nav-System dispersion and structural configuration tests passed 27 of 27.
  An extended four-file gate passed 40 tests; its sole failure occurred before
  the test body because legacy `test_navigation_update` still calls the removed
  `cosmica_setup` script.
- No Full-SLAM run was used for this verification; therefore the temporarily
  unavailable manoeuvre factors and guidance-disable workaround were not
  exercised.
