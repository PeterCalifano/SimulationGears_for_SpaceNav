# Native SRP inputs and construction

The full implementation checklist lives in the gtsam-space-nav `feature/srp-lut-bias` worktree,
at `doc/developments/native_srp_lut_development_plan.md`. This repository owns the panel law,
opaque shadow sampling and artifact construction; the estimator has no runtime dependency on it.

Build with the normal native CMake entry point. The standalone `srp_lut` program is controlled by
the existing `sim_gears_for_space_nav_BUILD_PROGRAMS` option. It supports:

```sh
srp_lut build spacecraft.srp.yml spacecraft.json 0.5329 5 transverse 64
srp_lut validate spacecraft.json
srp_lut inspect spacecraft.json
```

Construction accepts `scalar` or `transverse` channels and `off`, `64`, `256`, or `1024` shadow
samples. The C++ preparation API additionally supports subdivision levels 0 through 5. MATLAB's
panel law is used with frozen sampled visibility in direction partials; illumination terminators
are flagged nonregular. Shadow boundaries are discontinuous and are not differentiated.

OBJ/YAML schema versions 1 and 2 retain source triangle record order and winding. Every source
triangle must have exactly one front optical family. Version 2 rear references select the named
family's front coefficients; they do not recursively inherit rear assignments. Rear optical faces
reuse their source geometry, and occluders remain opaque on both sides. Closed components must
have positive oriented volume. The loader never repairs winding or triangulates polygons.

`SimulationGears.SrpResponseLut.v1` JSON contains these required fields:

| Field | Contract |
| --- | --- |
| schema | `SimulationGears.SrpResponseLut.v1` |
| frame, direction | `SCB`, `spacecraft_to_sun` |
| axis_units, area_units | `deg`, `m^2` |
| layout | `azimuth_elevation_channel` (channel fastest, then elevation) |
| azimuth, elevation | Uniform arrays spanning [-180,180] and [-90,90], exact endpoints; absolute uniformity tolerance 1e-10 deg |
| channels, channel_units | `[Cr]`, `[1]` or `[Cr,Tx,Ty,Tz]`, `[1,m^2,m^2,m^2]` |
| samples | Flat finite array; Cr nonnegative; nodal transverse response orthogonal to Sun direction |
| reference_area_m2 | Positive scalar area |
| provenance | String geometry_sha256/descriptor_sha256/generator, bool self_shadowing, uint32-compatible shadow_samples, finite nonnegative ray_offset_m |

Seam and pole values agree exactly. The table contains geometry/optical response, not pressure,
mass, attitude, gravity, biases, or ephemerides. Runtime reconstruction uses
`F/p = -area*Cr*d + (I-d*d')*T` with the true normalized body Sun direction `d`.
MATLAB export explicitly extracts the force subset from compact or fixed-capacity payloads;
optional torque samples are not transported into the estimator's force table.

The dedicated correctness tests are ordinary Catch2 tests. Timing comparisons and convergence
campaigns belong to later standalone benchmarks and are not registered with CTest.
