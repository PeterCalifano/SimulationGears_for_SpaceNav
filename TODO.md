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
- [x] Add `CShapeModel.setSphericalHarmonicsGravityData(...)` and `CShapeModel.getSphericalHarmonicsGravityData()` for explicit caching and retrieval.
- [x] Verify that the returned coefficients feed `EvalExtSphericalHarmExpInTargetFrame(...)` directly with no reshaping and document the expected storage format.
- [x] Keep the new fitting APIs free of `varargin` and aligned with the repo naming and documentation conventions.
- [x] Harmonize the SH evaluator stack around canonical target-frame and world-frame wrappers while preserving legacy `ExtSHE_*` entry points.
- [x] Rename geometric shell spacing to `ComputeLogSpacedValues(...)` so the logarithmic spacing behavior is explicit.
- [x] Remove the misleading combined acceleration-plus-potential relative-RMS fit metric and refine shells from the two metrics separately.
- [x] Convert `FitSpherHarmCoeffToPolyhedrGrav(...)` to positional inputs for codegen compatibility and update callers.
- [x] Extract mesh volume and center-of-mass computation into `ComputeMeshModelVolumeAndCoM(...)`.
- [x] Share the ExtSHE basis and spherical-gradient mapping implementation between fitting and evaluation instead of maintaining a duplicated fit-only model.
- [x] Add `BuildGravityModelMexTargets(...)` to build the ExtSHE and polyhedron gravity MEX targets.
- [x] Validate polyhedron gravity against the actual `/home/peterc/devDir/simulationUtils` implementation, including acceleration, Jacobian, potential, and Laplacian.
- [x] Add `CShapeModel.BuildAndSetSphericalHarmonicsGravityData(...)` as the instance-level build-and-cache API with default SH degree 4.
- [x] Add a shared shape-model scenario gravity-default registry for known-scenario `dGravParam` lookup.
- [x] Wire `DefineShapeModel(...)` to initialize SH gravity data by default when a loaded known-scenario shape has resolvable gravity defaults.

## MATLAB - Registry-Backed Scenarios And Shape Models

- [x] Introduce `CScenarioRegistry` as the shared source of truth for scenario aliases, SPICE names, fixed frames, physical constants, shape routing, reference geometry, and SH metadata.
- [x] Treat `EnumScenarioName.NotDefined` as passthrough/backward compatibility, and add `EnumScenarioName.FromShape` as the explicit custom shape scenario path.
- [x] Refactor `DefineShapeModel(...)` to consume registry specs while preserving its current first and second outputs and adding metadata only through an optional third output.
- [x] Refactor `CScenarioGenerator.LoadDefaultScenarioData(...)` to consume registry constants and replace the incomplete SH-loader placeholder.
- [x] Store registry SH metadata with both `ui32SourceMaxDegree` and `ui32HardcodedMaxDegree`, capped at degree/order 16 for checked-in payloads.
- [x] Use registry SH coefficients only when the requested degree is available; otherwise recompute from the loaded shape model when possible.
- [x] Keep public SH coefficient payloads in repo-native unnormalized `[Clm, Slm]` column-pair format with sufficient numerical precision and source/provenance metadata.
- [x] Add a scenario-driven dataset/environment builder on `CScenarioGenerator` with opt-in reference completion.
- [x] Add one offline regeneration script for deterministic registry literals and SH payloads; runtime code must not depend on it.
