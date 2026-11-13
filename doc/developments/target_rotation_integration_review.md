# Target rotation integration review

This note records the decisions and the open gates of the target rotation
integration. The work started on 11 October 2026 on `feature/harmonize-target-rotation`
and was followed up on 26 October. That branch is removed; its content is now
on `feature/target-rotational-state-models`.

## Decisions

- **Rate sign.** `dAngVel_IN` is the angular velocity of the target body frame
  relative to the inertial frame, in inertial coordinates. It satisfies
  `R_INfromTB(t) = Exp(+omega_IN*t) R_INfromTB(0)`. For SPICE sources,
  `cspice_xf2rav` on the inertial-to-target state transform already returns this
  rate, so `DefineEnvironmentProperties` stores it without a sign change.
- **Spin axis.** A SPICE frame attitude and rate declare no physical shape or
  inertia axis, so the SPICE path no longer writes a `DEFAULT_PLUS_Z` axis. The
  spin axis and its source travel as an optional pair and stay absent unless the
  source supplies them. The dataset path keeps its own contract, including the
  `DEFAULT_PLUS_Z` default of `SReferenceMissionDesign`.
- **Rate regime.** `enumAngularVelocityRegime` (`EnumTargetAngularVelocityRegime`)
  alone states whether rates exist. `EphemeridesDataFactory` checks it against
  every rate sample and no longer writes the first-sample `dNominalAngVel_IN`.

## Evidence

- `testSpiceRateSignAndOptionalAxisContract` compares the Moon `cspice_xf2rav`
  rate with centred differences of `cspice_pxform` and checks the absent spin
  axis and the known regime. It needs `SIMGEARS_SPICE_KERNEL_ROOT` and MICE.
- `testEphemeridesDataFactory` and `testDefineEnvironmentProperties` cover the
  regime checks, the optional spin-axis pair, the reuse of a dynamics structure
  and the `bLoadTargetRotationalState=false` path.
- The consumers in Nav-System, nav-backend, cosmica and gui-system already treat
  the spin-axis pair as optional and expect no `dNominalAngVel_IN` in their
  uncommitted trees (checked 13 November 2026).

## Open gates

1. Consumer contract checks in nav-backend and Nav-System
   (`target-rotational-state-consumers` worktrees), including the move of the
   propagated-prior consumers to the typed history.
2. Gravity, SRP, pointing and LiDAR regressions against the consolidated branch.
3. Standalone dependency closure: the tests so far loaded MathCore and the
   COSMICA ephemerides helpers from other checkouts.
4. Bounded simulation acceptance.
