# Lists of TODO SimulationGears_for_SpaceNav

## MATLAB

- [ ] Update unit tests for visibility check functions to ensure they cover all edge cases and scenarios.
  - [ ] Ellipse_localPA visibility check
  - [ ] Shadow rays based visibility check

## MATLAB - Polyhedron To Spherical Harmonics

- [x] Review, modernize, and validate the active spherical harmonics evaluation path, including MATLAB/MEX equivalence checks.
- [x] Reimplement and validate the self-contained polyhedron gravity preprocessing and evaluation path, including comparison against `simulationUtils`.
- [x] Add a standalone utility to fit spherical harmonics coefficients from the exterior polyhedron gravity field in the repo-native `[Clm, Slm]` column-pair format consumed by the canonical SHE evaluators.
- [x] Fit the perturbative field jointly on acceleration and potential, keeping degree-1 coefficients fixed to zero in the returned storage.
- [x] Add adaptive shell-based sampling with iterative refinement until the fit converges on a fixed dense validation set.
- [x] Derive or validate `dGravParam` / `dDensity` from the polyhedron volume and reject non-COM-centered meshes.
- [x] Add internal helpers required by the fitter, keeping naming, docs, and argument validation aligned with the current MATLAB conventions.
- [x] Add synthetic exact-recovery tests for the fitter using known SH coefficients.
- [x] Add end-to-end polyhedron-to-SH validation tests on representative meshes and verify error reduction with increasing degree.
- [x] Add a static `CShapeModel.BuildSphericalHarmonicsGravityData(...)` compute method returning a `strSHgravityData` struct.
- [x] Add `CShapeModel.SetSphericalHarmonicsGravityData(...)` and `CShapeModel.getSphericalHarmonicsGravityData()` for explicit caching and retrieval.
- [x] Verify that the returned coefficients feed `EvalExtSphericalHarmExpInTargetFrame(...)` directly with no reshaping and document the expected storage format.
- [x] Keep the new fitting APIs free of `varargin` and aligned with the repo naming and documentation conventions.
- [x] Harmonize the SH evaluator stack around canonical target-frame and world-frame wrappers while preserving legacy `ExtSHE_*` entry points.
