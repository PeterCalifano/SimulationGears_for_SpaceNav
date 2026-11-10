# Spherical-harmonics gravity

Run `SetupSimGears` after adding this repository's `matlab` directory to the
MATLAB path. The setup discovers the model and fitting folders. MATLAB resolves
the two lookup builders in `private` from their parent-folder caller; do not add
that private folder to the search path.

## Evaluation and model data

| Interface | Responsibility |
| --- | --- |
| `EvalExtSphHarmExpInTargetFrame` | Evaluate perturbing potential, acceleration and optional analytical position Jacobian in the body frame. |
| `EvalJac_ExtSphHarmExpInTargetFrame` | Preserve the Jacobian-only interface and error contract; delegate to the target-frame evaluator. |
| `EvalExtSphHarmExpInWorldFrame` | Rotate input position and output acceleration around the target-frame evaluator. |
| `EvalExtSphHarmSamplesInTargetFrame` | Evaluate a set of positions for fitting and diagnostics. |
| `BuildExtSphHarmExpBasis` | Build coefficient-linear potential and spherical-gradient matrices for least-squares fitting. |
| `EvalExtSphericalHarmExpCore` | Apply coefficients to that basis and expose spherical-gradient and Legendre outputs; retain the existing MEX interface and comparison implementation. |
| `MapSphericalGradientToCartesian` | Convert spherical-gradient outputs for the basis/fitting implementation. |
| `GetSphHarmNormalizationFactors` | Convert between normalized and repository-native coefficient conventions. |
| `RescaleSphericalHarmonicsReferenceRadius` | Change the coefficient reference radius while preserving the represented field. |
| `SSphericalHarmonicsGravityData` | Validate and serialize model data, units and coefficient conventions. |

The target-frame evaluator uses a Cartesian recurrence for the spherical-harmonic
potential and its first and second position derivatives. It remains regular at
either pole. Central gravity remains the caller's responsibility. Requesting only
potential and acceleration omits the second-derivative recurrence.

`private/buildSHElegendreLUT` supplies associated Legendre values, and
`private/buildSHEtrigLUT` supplies longitude terms and the latitude factor used
by `BuildExtSphHarmExpBasis`. The basis builder and gradient conversion remain
public because functions in the sibling fitting folder use them.

## Fitting

The following interfaces live in `../fitting`:

| Interface | Responsibility |
| --- | --- |
| `FitGravityFieldExtSHEcoefficients` | Fit coefficients to supplied potential and acceleration samples. |
| `FitSpherHarmCoeffToPolyhedrGrav` | Generate polyhedron reference samples and fit a harmonic model. |
| `RunFitSpherHarmonicsToPolyhedronGravityFromObj` | Handle OBJ input, fitting, saved products and plots. |
| `GenerateFibonacciSpherePoints` | Generate approximately uniform samples on one sphere. |
| `GenerateShellPointSet` | Assemble samples across several radii. |
| `ComputeGravityFieldFitMetrics` | Compare supplied predicted and reference fields. |
| `ComputePolyhedronGravitySHfitDiagnostics` | Evaluate independent holdout samples before computing fit metrics. |

## Compatibility and validation

`EvalCartesianExtSHE` is consolidated into the existing target-frame evaluator.
The unused `ComputedSpherdRSC`, `loadSHEcoeffModel` and `importSHEcoeffsFromFile`
interfaces are removed. Existing coefficient assets under
`accelerations/gravityModelsData` retain their locations.

The older complete evaluator `accelerations/.deprecated/SphericalHarmonics.m`
remains outside normal setup. Cross-validation loads it explicitly as a reference.
Tests retain their paths under `tests/matlab/simulation_models/accelerations`.

`testExtSHE_CrossValidation` checks raw acceleration finite differences,
independent polar limits, unit scaling, the public basis against MATLAB Legendre
values, and fixed/bounded MEX interfaces. `testFitSpherHarmCoeffToPolyhedrGrav`
checks coefficient recovery and polyhedron fitting. These checks establish
numerical consistency for their tested cases, not physical model calibration.

## Changelog

- 10 October 2026: Consolidate potential, force and Jacobian evaluation and group
  harmonic and fitting sources under gravity.
- 11 October 2026: Make the two lookup builders private and remove three unused
  legacy interfaces; retain coefficient data assets and the public fitting basis.
