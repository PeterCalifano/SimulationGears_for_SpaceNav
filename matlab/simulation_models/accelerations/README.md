# Acceleration models

Call `SetupSimGears` after adding the repository's `matlab` directory to the MATLAB path.
It discovers these subfolders and excludes `.deprecated`. Adding only the
`accelerations` directory no longer loads every model. Public function names
and numerical interfaces are unchanged by this reorganization.

| Folder | Responsibility |
| --- | --- |
| `gravity/point_mass` | Central and differential third-body acceleration and analytical Jacobians |
| `gravity/spherical_harmonics` | Evaluate harmonic gravity and manage its coefficients |
| `gravity/polyhedron` | Prepare and evaluate polyhedron gravity and gravity corrections |
| `gravity/fitting` | Sample, fit and assess harmonic approximations to gravity fields |
| `gravity` | J2 acceleration in the radial, along-track and cross-track frame |
| `srp/cannonball` | Cannonball SRP force and Jacobian |
| `srp/panels` | Panel preparation, optical response, visibility, force and Jacobian |
| `srp/lut` | Construct, validate and evaluate SRP response tables |
| `aerodynamic` | Atmospheric drag perturbations |
| `manoeuvres` | Impulsive errors and finite burns |
| `stochastic` | Gauss-Markov acceleration and sample sequences |

Keep `srp/lut/private/EvalSrpLutKernel.m` private to the LUT implementation.
The shared solar-pressure and cylindrical-shadow functions live directly under
`srp`. Builders remain under `matlab/builders`; tests retain their existing
paths under `tests/matlab/simulation_models/accelerations`.

## Spherical-harmonics interfaces

See [the harmonic model inventory](gravity/spherical_harmonics/README.md) for
public evaluators, private lookup builders, fitting utilities and removed legacy
interfaces. Keep the coefficient basis accessible to the sibling fitting folder.

## Data and generated products

Existing local assets under `gravityModelsData` stay at their original paths.
The two unused MATLAB loaders have been removed. Do not relocate saved
coefficient files, generated MEX products or caches as part of source organization.
Historical reports retain the paths used when their evidence was produced.

## Changelog

- 10 October 2026: Group existing sources by physical model; preserve numerical
  code, public names, data assets and test paths.
- 11 October 2026: Complete the layout after consolidating the harmonic models;
  document owner setup and preserve private LUT lookup scope.
- 11 October 2026: Consolidate the isolated point-mass and third-body kernels
  under `gravity/point_mass`; reuse them in dynamics and polyhedron corrections.
