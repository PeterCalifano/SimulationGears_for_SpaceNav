# Third-Body Gravity Sign Fix

**Status:** Implemented and focused-verified in the working tree; commit, release, and downstream dependency updates remain pending.

## Physical contract

For spacecraft position `r`, perturbing-body position `s`, and body gravitational parameter `mu`, all expressed in
the same main-body-centred inertial frame, SimulationGears shall evaluate

```text
a_3(r,s) = mu * ((s-r)/|s-r|^3 - s/|s|^3).
```

The affected implementation stored `r-s` and added the stored direct and indirect vectors. That expression was the
exact negative of the physical differential acceleration in both the generic-body loop and the separate Sun block.

## Source-owner implementation

- [x] Add an independent axis-aligned generic-body oracle that fails against the affected source with the exact
  opposite acceleration.
- [x] Add an independent arbitrary three-dimensional generic-body oracle.
- [x] Exercise the separately implemented Sun third-body block with an arbitrary three-dimensional oracle.
- [x] Exercise `evalRHS_InertialDynMaxFidelity` to prove the source-owner correction reaches its principal wrapper.
- [x] Correct the generic-body leading sign in `evalRHS_InertialDynOrbit`.
- [x] Correct the Sun leading sign in `evalRHS_InertialDynOrbit`.
- [x] Document the shared main-body-relative differential-gravity convention in the source header and formula block.

## Verification evidence

- [x] Before the production correction, all four new tests failed with `actual = -expected` for the physical oracle.
- [x] After the correction, all four new tests passed.
- [x] The focused third-body, max-fidelity, and SRP set passed 16/16 tests.
- [x] All four max-fidelity MEX variants compiled successfully from the corrected source: spherical-harmonic and
  polyhedron RHS/Jacobian targets.
- [ ] The complete legacy MATLAB suite is not clean in the current environment. Its attempted run reached unrelated
  failures caused by missing SPICE functions/data, obsolete external paths, and unavailable legacy setup helpers;
  no failure observed in the focused dynamics scope.
- [ ] Commit and release the SimulationGears source-owner correction under separate authorization.

## Downstream boundary

- [ ] Update standalone EstimationGears only through a reviewed SimulationGears revision; do not use a dirty nested
  checkout as a substitute for a dependency release.
- [ ] Update FUTURE-nav only after approved SimulationGears and EstimationGears revisions exist, then regenerate its
  checked-in C++ from MATLAB rather than editing generated sources.
