# SimulationGears worktree consolidation

29 September 2026. Consolidate the COSMICA-related provider changes in
`feature/consolidate-simulation-models`, using the single new worktree
`/home/peterc/devDir/SimulationGears_for_SpaceNav-worktrees/cosmica-upgrade-consolidation`.
Start from the latest local main-provider commit, `c6a030c`, which includes the
accepted reference-pressure and impulse-error changes. Review, test and stage one
functional batch at a time; leave commits to the user. Preserve active work,
unrelated indexes, dependency pointers and external assets.

## Verified inventory

Use the new worktree as the destination for Stages 2-3. The initial inventory
below was captured at `6c9db696` with an empty main index. Relative
commit counts and pending work below describe that snapshot. The user has since
committed Stage 1 at `1f10617` and Stage 1A at `c6a030c`; track subsequent changes
in the progress entries. Keep the other feature worktrees outside this migration.

| Worktree | Branch or checkout | Commit difference | Pending work |
| --- | --- | --- | --- |
| Main checkout | `feature/upgrade-filter-smoother-interface` | Destination | OBJ selection/gravity, kernel utilities, panel visibility and SRP LUT source |
| `phase-d-actuation-staging` | Detached at `e86d14d8` | One commit behind | Two staged files; direction-error correction is absent from main |
| `phase-d-kernel-review-staging` | Detached at `e86d14d8` | One commit behind | Seven staged files and two untracked panel-visibility files |
| `target-rotational-state-models` | `feature/target-rotational-state-models` | One unique commit; 14 main commits absent | 20 modified/untracked files; generated histories and typed rotation contracts |
| `landing-building-blocks` | `feature/landing-building-blocks` | One commit behind | 59 modified/untracked files; B00-B04 spacecraft and dynamics building blocks |
| MSCKF isolation checkout | `feature/msckf-revamp-isolation` | Four commits behind | 14 modified/untracked files; gravity kernels and consumers |
| Missing CMake checkout | `feature/cmake-template-v1.9-from-develop` | Branch is an ancestor of main | Directory already absent; stale registration pruned |

The main and rotation branches have configured upstreams. Landing and MSCKF use
local branches. The detached checkouts were created to protect running campaigns;
they contain uncommitted review batches, rather than abandoned copies.

An upstream records the remote branch for fetch/pull comparisons. It does not
update a worktree when another branch changes. Nested SimulationGears submodules
also intentionally check out the commit recorded by their parent repository;
update those pointers only after the consolidated provider passes consumer checks.

SSH fetch failed with a connection reset. An HTTPS remote-ref check confirmed
`develop` at `e139335c` and `feature/upgrade-filter-smoother-interface` at
`e86d14d8`. The new branch starts from the newer reviewed local provider commit;
this migration does not merge remote history or alter dependency pointers.

## Stage 0: Protect and classify

- [x] Inspect every registered worktree, HEAD, upstream, index and dirty path.
- [x] Compare detached staged files with their main-checkout counterparts.
  Retain the kernel loaded-file identity fix and the small-burn correction;
  both are missing from main. Keep the unique panel-visibility test.
- [x] Save the inventory, index/worktree patches and source hashes under
  `/tmp/simgears-worktree-consolidation-20260929`.
- [x] Check processes. The RCS-1 adaptive-centroid MATLAB campaign remains live;
  do not interrupt it or change its input/dependency checkouts.
- [x] Prune only the missing CMake worktree registration. Verify its branch,
  every surviving HEAD, existing index and dirty source hash remain unchanged.
- [x] Refresh remote references through HTTPS after the SSH connection failure.

## Stage 0A: Relocate the COSMICA provider work

The user explicitly requested a new feature worktree and removal of
`phase-d-kernel-review-staging`, including its SPK, SRP and OBJ work. This replaces
the earlier destination choice and, for this stale worktree only, permits removal
after verified preservation without waiting for a commit. Do not remove another
feature worktree under this exception.

- [x] Pin the destination to `c6a030c` and create one worktree on
  `feature/consolidate-simulation-models`; leave the main branch unchanged.
- [x] Archive the main candidates and all nine stale-worktree files, including
  separate index blobs, binary patches, source hashes and worktree inventory.
  Keep the recovery archive outside Git at
  `/tmp/simgears-cosmica-migration-20260929-lw58q4xp`.
- [x] Transplant the 38 main source, test and metadata candidates through an
  explicit path list. Exclude editor settings, another thread's PR plan, runtime
  simulation configs, fetched kernels and large assets.
- [x] Retain the newer main SPK implementations and tests. Reconcile every
  stale overlap and copy the unique generic panel-visibility test.
- [x] Verify copied file hashes and API dependencies. Check function resolution
  from the new worktree and run focused SPK, geometry and SRP source checks.
- [x] Check launch references and process working directories before removing
  the stale path. Verify its complete index and untracked contents are preserved,
  clear only that verified file list and use normal worktree removal.
- [x] Review and stage the next coherent batch in the new worktree. Leave the
  remaining geometry and SRP candidates unstaged until their own review gates pass.
- [x] Recheck the main checkout and other worktree indexes. Preserve any donor
  copies required by the live RCS-1 campaign; clean them only after safe provider
  integration. Record retained duplicates explicitly.

## Stage 1: Consolidate impulse execution errors

- [x] Review the isolated two-file correction against main, preserve its
  small-angle model and remove the extra impulse-magnitude multiplication.
- [x] Replace the Phase-D-specific regression with a generic angular-dispersion
  check across impulse magnitudes. Preserve the existing behavioral tests.
- [x] Review function/class documentation, imperative comments, naming and
  formatting. Run the focused suite, Code Analyzer and whitespace checks.
- [x] Stage only the reviewed implementation, tests and this plan in main.
  Inspect the complete index and suggest an imperative commit message.
- [x] Wait for the user's commit; verify the committed source preserves the
  isolated correction before considering worktree removal.

### Stage 1A: Extend impulse errors to finite angles (implemented)

The user approved implementation and staging on 29 September. Keep the extension
separate from the committed first-order correction; leave the commit to the user.

Retain the existing independent draws: fractional magnitude error
`epsilon_m = sigma_m * randn` and signed rotation angle `theta = sigma_theta * randn`.
Retain the perpendicular random-axis selection and draw order. Apply
`realized_delta_v = (1 + epsilon_m) * R(theta * axis) * nominal_delta_v`, using
MathCore's existing active `RotationVectorToDCM`. Reuse that implementation;
introduce no additional rotation helper, model enum or small/large-angle switch.

For direction-only errors, rotation preserves impulse magnitude. The previous
tangent approximation instead produces magnitude ratio `sqrt(1 + theta^2)` and
angle `atan(abs(theta))`. At a 30-degree draw, those are approximately 1.129 and
27.64 degrees. The exact model produces unit magnitude ratio and 30 degrees.

Keep `sigma_theta` as the standard deviation of the unwrapped signed draw.
The principal pointing error lies in `[0, pi]` and wraps large draws; its RMS is
not generally `sigma_theta`. Preserve the current Gaussian signed magnitude
factor, including its existing negative-factor behavior. A positive-only
magnitude distribution or another directional distribution requires a separate
noise-model decision.

- [x] Agree on the finite-angle contract before implementation.
- [x] Replace the tangent addition with the existing exact active rotation and
  magnitude factor. Preserve public arguments, RNG draws and the zero-impulse guard.
- [x] Test direction-only norm preservation, zero dispersions, velocity-unit
  invariance and deterministic finite-angle oracles through the actual function.
- [x] Check non-axis-aligned impulses and small-angle convergence. Use independent
  sine/cosine oracles at finite angles rather than validating MathCore against itself.
- [x] Update ensemble expectations: for zero magnitude dispersion, the Gaussian
  rotation model has mean vector `exp(-sigma_theta^2 / 2) * nominal_delta_v`.
  Test wrapped principal angles without claiming their RMS equals the draw sigma.
- [x] Run focused source and fresh fixed-size MEX checks; review full source/API
  documentation, formatting, comments and unnecessary complexity.
- [x] Stage only the approved implementation, tests and this plan; review the
  complete index and preserve all other worktree sources and indexes.
- [x] Verify the user's extension commit at `c6a030c` before the next batch.

## Stage 2: Consolidate selected kernel bundles and segmented states

- [x] Review the seven-file stale kernel batch against the newer main source.
  Preserve loaded-path multiset validation, the same-count wrong-set regression,
  relative asset-root resolution and caller-pool preservation on hash failure.
  Keep segmented-state and manoeuvre extraction behavior unchanged.
- [x] Keep bundle identifiers and asset metadata in manifests. Keep utilities
  independent of Apophis/RCS-1 and leave SPK/large asset payloads external.
- [x] Run bundle and segmented-extraction suites with real MICE setup; report
  missing-asset skips separately from passes. Complete the source and documentation
  review; all ten checks pass without incomplete tests or Code Analyzer findings.
- [x] Stage the reviewed SPK implementation, tests, manifest and documentation;
  inspect the complete index and leave geometry/SRP source outside this batch.
- [ ] Wait for the user's commit and verify the reviewed implementations survive
  in the destination. Apply the explicit Stage 0A preservation exception when
  removing the stale kernel checkout before that commit.

### Stage 2A: Reuse the shared MathCore hash package

The user requested ownership of file hashing in MathCore. Keep bundle selection
and SPICE-pool validation here. Preserve the `ComputeFileSha256` caller interface
through a thin MathCore adapter to the existing DataHash implementation.

- [x] Inspect existing providers and compare DataHash file-mode digests with the
  earlier helper for empty, text, binary and multi-chunk files.
- [x] Copy the installed DataHash 1.7.1 source, tests and BSD license into
  MathCore's `matlab/misc/DataHash`. Keep runtime source and license unchanged;
  record the test-fixture encoding correction. Exclude installer metadata,
  archives and screenshots; document provenance and an executable example.
- [x] Place the typed file adapter in MathCore's `matlab/misc` and remove the
  SimulationGears helper. Preserve the existing COSMICA, kernel and SRP callers.
- [x] Run the upstream package suite, file-integrity checks and actual bundle,
  segmented-state and SRP callers with the pending MathCore provider on the path.
  Check function resolution explicitly so an installed add-on cannot mask a gap.
- [x] Review and update the MathCore and SimulationGears indexes. Preserve the
  independently edited loader layout and leave unrelated geometry/SRP work unstaged.
- [ ] Obtain the user's MathCore commit, then update the SimulationGears dependency
  revision and rerun the consumer check with normal `SetupSimGears` alone before
  integrating the provider. Do not change the gitlink during this source-only batch.

### Stage 2B: Consolidate documentation entry points

Use the existing `doc/main_page.md` for maintained MATLAB interfaces and this
plan for status, pending work and dated validation evidence. Remove the standalone
SPK/OBJ notes and the SRP source-folder README after preserving their contents.
Use MathCore's existing main page for the shared hash package documentation.

- [x] Inventory related notes, current indexes and links; preserve source files
  and unrelated development plans before editing.
- [x] Consolidate SPK, OBJ-selection and SRP-table interfaces and examples in
  the SimulationGears main page; add a link from the repository README.
- [x] Move historical OBJ qualification, baseline failures and fit-cost evidence
  into this plan, retaining dates and the full-fit approval boundary.
- [x] Fold DataHash usage, provenance, license and test-compatibility notes into
  MathCore's main page; remove the separate package README.
- [x] Check links, examples, heading hierarchy, units and removed-path references.
  Apply the documentation readability pass and verify source bytes stay unchanged.
- [x] Update the reviewed indexes. Keep the OBJ/SRP guide sections unstaged with
  their pending implementations; preserve the separate MathCore dependency gate.

## Stage 3: Consolidate geometry and SRP work in separate related batches

- [ ] Preserve the main-checkout panel-visibility implementation; reconcile the
  older review copy, bring over its missing generic test, and validate geometry.
- [ ] Review OBJ object selection and gravity-fitting changes with their owning
  renderer/asset plan. Preserve source units, selected-object identity and parser
  behavior. Keep full gravity fitting and asset delivery under that plan's gates.
- [ ] Review the SRP LUT generator, kernels, analytical derivatives, codegen and
  tests after its active timing stage finishes. Reuse the existing ownership
  split: SimulationGears force mathematics, EstimationGears filter adapters.
- [ ] Stage each completed functional batch with its tests/documentation;
  preserve any work still active in another thread.

### OBJ qualification record (24 September 2026)

Retain this dated record from the supplied OBJ-selection note. It qualifies
macro selection and bounded gravity preparation; it does not close Stage 3,
register the Apophis asset or qualify the full degree-16 fit. Keep current
remaining work in the Stage 3 checklist above.

#### Qualification

- [x] Ten synthetic tests cover defaults, exact/repeated/unnamed objects, unions and missing names,
  material/group independence, compaction before repair and unit/radius evaluation, independent
  UV/normal indices, relative polygons, excluded malformed faces, bounded decoder crossings,
  non-OBJ rejection, builder metadata, selected-solid fit equivalence and scenario-generator
  forwarding with registered GM/reference radius.
- [x] All fixtures use disposable local files; no external asset directory is required. The
  scenario-generator test uses the tracked manifest with an explicitly supplied temporary root.
- [x] MATLAB Code Analyzer reports zero messages across all nine touched/new MATLAB files.
- [x] Broader focused regression: 42 passed, six failed. Untouched HEAD reproduces the same six
  failures (32 passed). These are existing manifest-routing fixture/expectation failures, not a
  green overall suite. See the failure list below.
- [x] Separate read-only acceptance on the supplied full-resolution OBJ selected 4,157,440 faces
  and 2,078,722 vertices, matching an independent source-record count. No repair or simplification.
- [x] Bounded gravity preparation/three-point evaluation passes the existing closure/shared-edge
  winding checks and produces finite dyadics/fields. Full-resolution cost estimate recorded below.
- [ ] Finish asset identity, frame/origin declarations, consistent render/gravity transforms,
  full-resolution SH16 qualification, coefficient embedding and activation.
- [ ] TODO: geometric self-intersection detection remains deferred. Closure, orientation and
  finite-volume checks do not prove a non-self-intersecting solid; callers retain that contract.

Run the independent tests from this repository:

```sh
matlab -batch "run('matlab/SetupSimGears.m'); results = runtests('tests/matlab/simulation_models/testObjObjectSelection.m'); assert(all([results.Passed]));"
```

The six reproduced baseline failures are:

- `testCShapeModelSimplifyMesh/testDefineShapeModelPassesLoadTimeKeepFraction`
- `testCShapeModelSimplifyMesh/testDefineShapeModelInitsSHGravityByDefault`
- `testCShapeModelSimplifyMesh/testDefineShapeModelMoonDefaultShapeUsesSimGearsDataRoot`
- `testCShapeModelSimplifyMesh/testDefineShapeModelMissingManifestShapeFailsEarly`
- `testCShapeModelSimplifyMesh/testDefineShapeModelUsesExplicitApophisElongatedScenario`
- `testDefineShapeModel/testUsesRegistryReferenceDefaults`

#### Supplied-source evidence

Read-only source: `/home/peterc/Downloads/apophis_boldered_v01/apophis_generated_v1/apophis_generated_v1.obj`.
SHA-256: `840b184ccd75d1b0d033ebe95d6c32f47d9b0540628b57e3314311e870bb0edc`.
The eventual asset-store copy is still pending; no input was modified or committed.

After explicit m-to-km conversion, the selected macro geometry gives:

- Signed-tetrahedron volume: `0.017050140819315635 km3`.
- Source-origin volume centroid: `[-0.00025751207144790065, 0.000040325690992033907,
  -0.00067342156994870336] km`.
- Enclosing radius about that centroid: `0.24482689297731444 km`.
- Recomputed centered centroid norm below `1e-10 km`, with unchanged volume.

These are geometry-selection/centering checks, not SH16 accuracy, source-frame alignment or
self-intersection certification. The registered reference radius is not replaced by the
enclosing radius.

Logs are retained in the renderer worktree's ignored `build/` directory:
`sh_object_selection_verified_tests.log`, `sh_object_selection_final_regression.log`,
`sh_object_selection_baseline.log`, `sh_object_selection_code_analyzer.log` and
`apophis_sh_selection_acceptance.log`. The baseline was an archived copy of SimulationGears
HEAD `e86d14d8bbab73ef79ac60f1b2c870c83072706e`, with unchanged MathCore sources on the MATLAB path.

#### Full-resolution fit cost boundary

The bounded check uses the unchanged `ComputePolyhedronFaceEdgeData` and
`EvalPolyhedronGravPerturbationSamples` implementations. The selected macro mesh has 6,236,160
unique edges; preprocessing took 11.81 s. All edges passed the existing exactly-two-faces and
opposite-traversal checks, signed volume was positive, and all dyadics were finite. This does
not check self-intersections or establish the source's physical frame orientation.

Three exterior field samples at 1.1, 2.0 and 3.5 times the centered enclosing radius took 1.039,
0.891 and 0.897 s respectively. At degree 16, the current fitter uses 285 unknowns, 5,130 initial
training samples and 8,550 validation samples. Scaling the median time projects about 12,266 s
(3.4 hours) for those field evaluations alone. This is a rough cost estimate, not a benchmark or
promised runtime: it excludes the SH solve/evaluation and adaptive refinement, and each probe
call checks its input arrays independently while a full sample batch validates them once.

No full fit was launched. The user selected review of an optimization proposal before the full
run. Prepare that proposal first; origin/provenance setup remains required as well. No sampling
reduction, mesh decimation, solver change or alternative dependency is authorized by this choice.
Evidence: renderer `build/apophis_sh_cost_probe.log`; the entire disposable probe was bounded
by a 180 s process deadline and completed successfully.

## Stage 4: Reconcile target rotational-state models

- [ ] Compare the unique `44f0d19` covariance/inertia commit with main and reuse
  its implementation without introducing duplicate commits.
- [ ] Reconcile the 14 newer main commits with the dirty rotation work. Prepare
  the exact integration for review before any history or dependency changes.
- [ ] Check the corrected MathCore integration prerequisite, active attitude
  matrices, angular-velocity regime enum, covariance propagation and existing
  COSMICA consumers before exposing the new runtime payload.
- [ ] Run focused MATLAB/MEX rotation, sampling, environment/factory and real
  consumer contracts. Record blocked external-asset checks explicitly.
- [ ] Review and stage coherent completed batches; wait for user commits and
  integration authorization before closing the rotation worktree.

## Stage 5: Consolidate landing building blocks

- [ ] Review B00-B04 source, wrappers, examples and validation documentation as
  coherent library batches. Preserve the restricted momentum-flux assumptions;
  keep B05+ and COSMICA landing integration outside this consolidation.
- [ ] Run native tests, Python/MATLAB binding checks and installed-consumer
  checks appropriate to each batch. Distinguish baseline asset-fetch failures
  from numerical model qualification.
- [ ] Keep existing CMake/wrapper infrastructure and dependency pointers intact
  unless an identified integration prerequisite requires separate authorization.
- [ ] Stage, obtain user commits, verify consolidated evidence and preserve
  useful ignored validation reports before removing this worktree.

## Stage 6: Reconcile MSCKF gravity providers with the isolation stack

- [ ] Compare point-mass/third-body helpers, polyhedron corrections and dynamics
  consumers with main. Remove duplicate implementations only after checking
  contracts and codegen behavior.
- [ ] Review and validate the SimulationGears batch with the related MSCKF
  consumer changes. Preserve that stack's ownership and existing isolation.
- [ ] Close the worktree only after its unique work is committed and the
  dependent isolation stack no longer requires this checkout.

## Stage 7: Remove obsolete worktrees and verify consumers

- [ ] For each removal, verify committed source coverage, an empty index and
  working tree, no remaining callers/processes, and preserved useful evidence.
  Apply the explicit Stage 0A exception only to the stale kernel worktree. Never
  use force removal to bypass an unreviewed dirty checkout.
- [ ] After verifying committed coverage, clear only duplicate review changes
  through a path allowlist before normal worktree removal. Keep unique untracked
  source/tests and useful validation evidence until their own batches are committed.
- [ ] Remove the two detached review checkouts first after Stages 1-3 preserve
  their source and missing tests. Retire other feature worktrees after their own
  consolidation and consumer gates pass.
- [ ] Review editor workspace entries and launch/setup references before
  deleting paths. Preserve references needed by active work.
- [ ] Remove obsolete branches only after explicit authorization. Preserve
  parent-pinned dependency checkouts until their separate pointer review.
- [ ] Recheck the remaining worktrees, all affected indexes, dependency
  consumers and the existing Nav-Backend pressure batch. Report exact removals,
  retained work and validation limits.

## PR #11 technical-debt follow-up

29 September 2026. Address five defects: dataset rate loss, missing RCS rate
metadata, OBJ record truncation, incomplete Python package versions and discarded
Docker build arguments. Append the execution sequence here; use the main page
for maintained interface documentation. The user approved this worktree and
requested checking existing implementations before editing. The five fixes are
implemented and pass focused tests. Review and stage the angular-velocity batch
first; keep the OBJ and packaging/container batches for subsequent user commits.

### Fix stage 1: Confirm ownership and preserve the baseline

| Reference | Current observation | Treatment |
| --- | --- | --- |
| Active consolidation worktree | `feature/consolidate-simulation-models`; initial `c6a030c`, now `e3e1e0d` after the external SPK commit | Review the rate-transport batch first; preserve pending OBJ/SRP source |
| Original PR source | `feature/extend-support-for-space-nav-backend` at `801b0b7` | Already an ancestor of the current branch; replay no original commits |
| Audit destination | Landing worktree at `e86d14d` | Keep as a separate integration option; retire `c9a99db` as an execution baseline |
| SimulationGears MathCore reference | `lib/MathCore_for_ComputerVision` at `e3a39c68` | Preserve it; do not restore `83d9d7b` |
| Canonical MathCore | Clean `develop` at `5bc1ccf`; shared hashing committed | Keep its separate dependency-update gate in Stage 2A |
| Template donor | `main` at `b7dc26c`; helper and focused test now modified | Leave both edits unstaged and uncommitted |
| [PR #11](https://github.com/PeterCalifano/SimulationGears_for_SpaceNav/pull/11) | Closed; head still `801b0b7`; no attached checks | Treat review threads as history, not evidence of completed fixes |

These observations replace the quoted audit's empty-index and open-PR
assumptions. Refresh them before implementation; a saved SHA is not permission
to overwrite newer work. Preserve the current source, document and index snapshots
under `/tmp/simgears-pr11-followup-plan-20260929-ijor5e4b`.

- [x] Inspect branch ancestry, dependency references, donor state and the five
  affected source paths. Confirm the original PR is already in the current branch.
- [x] Archive current source/document hashes and staged/working patches outside Git.
- [x] Use the existing consolidation branch, as approved. Replay no original
  PR commits and create no additional worktree.
- [x] Preserve the nine-file SPK index until its external commit at `e3e1e0d`.
  Confirm its content before preparing the next reviewed batch.
- [x] Refresh active-process dependencies and overlapping edits before changing
  source; keep protected campaign providers and other worktree indexes untouched.

### Fix stage 2: Preserve angular velocity through dataset and ephemeris adapters

- [x] Forward `dTargetAngVel_IN` in
  `SReferenceImagesDataset.FromSReferenceMissionDesign`. Trace the other existing
  dataset conversions and preserve a supplied finite 3-by-N sequence, including
  zero values and time-varying rates, without deriving it from sampled attitudes.
- [x] Extend `EphemeridesDataFactory` so both interpolation layouts retain the
  existing `strMainData.strAttData` rate metadata: `dAngVel_IN`,
  `dNominalAngVel_IN` and `dAngVelTimegrid`. Share the metadata assignment outside
  the layout branch; keep the RCS coefficient fields and existing validation.
- [x] Align each rate sample with the selected interpolation time domain.
  Document rates in radians per second and timegrid scaling explicitly; preserve
  the existing absolute/relative and seconds/days choices.
- [x] Clarify the SPICE comment in `DefineEnvironmentProperties` from the
  [NAIF contract](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/MATLAB/mice/cspice_xf2rav.html).
  `cspice_xf2rav` expresses its vector in the transform's input frame, already
  inertial for the current call. Preserve the transform direction and the
  onboard-model sign; add no second frame rotation.
- [x] Extend the dataset conversion and ephemeris-factory suites. Cover supplied,
  absent, zero, time-varying and invalid rate sequences, timestamp alignment,
  unchanged non-rate fields, both interpolation layouts and the frame/sign contract.
- [x] Exercise the real external RCS helpers with double polynomial degrees;
  assert their resolved paths and report missing dependencies as incomplete
  validation. Do not use a fit stub as evidence of RCS integration.
- [x] Record the separately reported `uint32` helper incompatibility with its
  owning function and focused reproduction. Keep it outside the rate-preservation
  fix unless a demonstrated prerequisite requires a separate approved correction.

### Fix stage 3: Reconcile OBJ whitespace handling with object selection

- [x] Modify the pending `LoadShapeMesh` implementation in place. Recognize
  leading spaces and tabs consistently for vertex and face records in the fast
  reader, selection scan and fallback eligibility checks.
- [x] Prevent a successful fast-path result from silently omitting valid records.
  Preserve source face order, index meaning, selected-object identity and the
  existing fallback for slash indices, relative indices and polygons.
- [x] Extend the existing OBJ tests with a closed four-face fixture containing
  mixed indentation. Verify complete geometry for default loading and explicit
  selection, including indented vertices, object declarations and fallback faces.
- [x] Check repair on/off, winding, bounds and volume with synthetic geometry.
  Run the existing object-selection and shape suites; preserve their documented
  public entry-point contracts and record baseline failures separately.
- [ ] Review the combined selection and truncation fix as the geometry batch
  under Stage 3. Update its main-page contract and stage the source/tests together;
  do not split overlapping parser hunks into incompatible commits.

### Fix stage 4: Preserve package versions and custom Docker arguments

- [x] Replace `@PROJECT_VERSION@` in SimulationGears' `python/pyproject.toml.in`
  with `@PYTHON_PACKAGE_VERSION@`. Reuse the existing CMake version composer;
  add no second conversion implementation.
- [x] Check generated metadata for release, prerelease and build-metadata inputs
  using disposable CMake/Python consumers. Verify valid Python version syntax and
  agreement with the existing composer; avoid a full wrapper build for this fix.
- [x] Restore the existing `b15b760` argument-preservation logic in the template
  donor's `.devcontainer/update_devcontainer_json.py` first.
  Preserve custom `build.args`; update only `ROS_MODE`, `ROS_DISTRO` and
  `ROS_PROFILE` when ROS is enabled and remove only those keys when disabled.
- [x] Import the reviewed shared change into SimulationGears without dropping
  repository-specific behavior. Keep the donor patch and its focused tests
  unstaged, uncommitted and unpushed.
- [x] Test both helpers against temporary JSON/JSONC fixtures with custom build
  arguments. Cover ROS disabled, enabled, on/off transitions, repeated runs,
  empty arguments and preservation of unrelated build/container fields.
- [x] Run Python syntax checks and the relevant existing helper checks. Add no
  tests tied to tunable simulation profile values and require no Docker image build.

### Fix stage 5: Review, stage and verify integration

- [ ] Prepare three related review batches: angular-velocity transport, combined
  OBJ selection/parser work, and packaging/container preservation. Keep unrelated
  SRP, landing, target-rotation and configuration changes outside those indexes.
- [ ] Apply the staged-code quality gate: review public documentation, imperative
  comments, logical blocks, naming, readability and unnecessary complexity.
  Run focused tests, Code Analyzer where applicable and cached whitespace checks.
- [ ] Stage each SimulationGears batch through an explicit path/hunk list, review
  its complete index and suggest a repository-style subject ending in `(Codex)`
  plus imperative body bullets. Stop for the user's review and commit between batches.
  - [x] Review and stage angular-velocity transport, dataset unit-label preservation,
    the SPICE comment, focused tests and the matching main-page section.
  - [ ] Review and stage combined OBJ selection/parser work.
  - [ ] Review and stage packaging/container preservation; retain donor edits unstaged.
- [ ] If the original source branch is chosen, integrate only the new user-created
  fix commits after separate authorization. Prefer compatible destination changes
  and reconcile its newer ephemeris and OBJ-selection implementation explicitly.
- [ ] Refresh the integration target and rerun the focused contracts against the
  final combined source. Keep `e3a39c68` unless the separate MathCore dependency
  update is approved; qualify that update independently rather than restoring an
  obsolete reference during a merge.
- [ ] Record final hashes, outcomes, remaining limitations and donor dirty/index
  state in this plan. Require no full simulation for these fixes; leave merge,
  push and PR operations to separately authorized steps.

## Progress and discrepancies

- 29 September, rate-transport staged review (`Codex gpt-6`): Recheck the empty
  index and stage five MATLAB source/test files, the dataset-rate main-page
  section and this follow-up plan. Replace class-header placeholders with the
  actual data contract and use imperative comments for state, camera and unit
  handling. Review complete staged files and verify the source matches the
  reviewed working copy. Pass seven fresh focused contracts with real RCS helpers;
  retain only the documented baseline analyzer finding. Keep OBJ/SRP, packaging, asset-root setup, guidelines,
  donor changes and dependency references outside this batch.
- 29 September, existing-fix audit and implementation (`Codex gpt-6`): Inspect
  registered worktrees, local/cached branch versions and the template donor
  before editing. Find Docker argument preservation in SimulationGears
  `b15b760`, subsequently regressed by template import `ee6d0c4`. Restore only
  that reviewed block in the donor, then copy its helper and focused test here
  unchanged. Reuse the rotation worktree's common rate-transport placement and
  stale-field cleanup; retain this branch's three-field/first-sample contract
  and model sign. Its later positive-spin convention, removed nominal field
  and typed rate regimes remain separate target-rotation work.
- 29 September, validation: Pass 28/28 MATLAB contracts: two image-dataset,
  five ephemeris-factory, twelve object-selection and nine mesh-reader tests.
  Resolve the real RCS helpers explicitly and use double polynomial degrees.
  Pass eleven Python checks here and seven in the donor, covering package
  versions and argument preservation. Introduce no simulation-config value
  tests. Six changed MATLAB files have no analyzer findings; the existing
  unused third-body frame output in `DefineEnvironmentProperties` remains
  outside this fix; compare its analyzer message with the committed baseline.
  Pass a real-MICE `rav2xf`/`xf2rav` round-trip using a nonidentity rotation to
  verify the documented frame relationship. Run no full simulation, wrapper
  build or Docker image build.
- 29 September, external helper discrepancy: Calling `EphCoeffsGeneration`
  with `uint32(3)` reaches `chebCoeffsGeneration.m:14` and fails at `cos` because
  its angle expression retains the integer type. The identical input with
  degree `3.0` succeeds. Preserve the external implementation and record the
  reproduction in `supporting-contracts.log`; treat a helper correction as
  separate work, not part of rate transport.
- 29 September, small discrepancies: MATLAB's `functiontests` entry does not
  permit an Output validation block; keep typed blocks on ordinary helpers.
  Correct fixture use of the mission constructor and allow repair to renumber
  vertices when comparing ordered triangle coordinates. The rerun exposes
  a pre-existing length-unit-label loss in the same dataset converter; preserve
  that label with the supplied rates without changing numerical state units.
  Retain the original failed logs beside successful evidence under
  `/tmp/simgears-pr11-existing-fixes-audit-20260929-q1szek4h`.
- 29 September, concurrent consolidation before follow-up staging: The existing SPK index is committed
  externally as `e3e1e0d` during validation. Preserve that commit and the separate
  edits to `ResolveSimulationRenderingAssetsRoot` and `DefineShapeModel` which
  appear meanwhile in this worktree and the main provider. Verify the SPK commit
  exactly matches the protected nine-file index. Keep the rotation worktree,
  MathCore, all remaining HEADs and other indexes unchanged. Leave all follow-up
  fixes and donor changes unstaged; make no agent commit, merge or dependency update.
- 29 September, initial source review before staging: Review complete changed methods and reader/helper
  contracts. Add public API documentation, separate logical blocks and purpose
  comments; remove the implementation-history comment from the rate assignment.
  Add `Codex gpt-6` to new changelog entries without rewriting earlier credit.
  Verify helper/test byte equality across provider and donor, Python syntax,
  working/cached whitespace and the source-path allowlist. Stage no new batch;
  the full index review remains part of the user's next consolidation step.
- 29 September, documentation harmonization: Remove three standalone
  SimulationGears notes and the MathCore package README after preserving them
  in the documentation-harmonization archive. Use the existing main pages for
  SPK, OBJ, SRP and hashing contracts, with repository README links. Move dated
  OBJ test results, baseline failures and bounded-fit evidence into Stage 3 of
  this plan. Keep the pending OBJ/SRP guide sections unstaged with their source;
  stage the SPK section and shared hash documentation for the current batches.
  Verify nine local links/anchors, balanced example fences, unchanged SPK/hash
  examples and unchanged source/configuration hashes. Update the indexes to
  seven MathCore files and nine SimulationGears files. Change no algorithm,
  configuration, asset or dependency revision; run no further simulation or
  native documentation build for this documentation-only change.
- 29 September, hash reuse: The earlier reuse audit missed the installed DataHash
  add-on. Verify matching file-mode digests, import the existing package into
  canonical MathCore and replace the custom loop with a typed adapter there.
  Remove the SimulationGears helper; preserve existing caller names and the main
  donor checkout used by the live campaign. Wait for the MathCore commit before
  changing the SimulationGears dependency revision.
- 29 September, upstream-test compatibility: The original DataHash test writes
  `char(0:255)` through `fwrite`, producing 384 UTF-8 bytes on this MATLAB release
  while expecting the digest of 256 raw bytes. Confirm this with independent
  byte and digest checks. Change only that test-fixture write to explicit uint8
  bytes, with a local-history note. Keep DataHash runtime source and license
  unchanged. Preserve the original failed log in the hash-consolidation archive.
- 29 September, shared-provider validation: Pass the corrected upstream suite,
  including 1,000 randomized format checks, and five independent Python SHA-256
  file oracles. Pass all ten SPK consumer tests with real MICE and installed
  external kernels; no tests are incomplete. Build a bounded SRP fixture and
  verify its geometry and generator digests against independent file hashes.
  Confirm both hashing entry points resolve to canonical MathCore and all six
  reviewed MATLAB files have zero Code Analyzer findings. Validate with pending
  MathCore explicitly on the path; normal submodule-only setup awaits its commit
  and dependency update. Run no full simulation or native build for this change.
- 29 September, revised staged handoff: Stage six MathCore package/adapter files
  and revise the SimulationGears SPK batch to eight files. Remove the helper's
  staged addition here; retain the independently edited loader layout and remove
  its trailing whitespace. Review both complete indexes, API documentation,
  imperative comments and dependency ownership. Preserve upstream formatting
  through file-local whitespace attributes and change only the documented test
  fixture. Leave the main donor index, geometry/SRP candidates, runtime configs,
  external assets and dependency pointers untouched. Obtain the MathCore commit
  first; do not treat the pending dependency update as completed validation.
- 29 September, relocation: Create the requested feature worktree at `c6a030c`.
  Copy 38 selected main files and the stale worktree's unique panel-visibility
  test with verified hashes. Retain the newer main SPK implementations, including
  relative-root handling and nonempty caller-pool preservation. Keep the exact
  stale working files and index blobs in the recovery archive before removal.
- 29 September, dependency setup: Initialize only the new worktree's recorded
  MathCore submodule at `e3a39c68`, cloning existing local objects. Change no
  provider checkout, dependency commit or parent gitlink. Verify ten provider
  entry points resolve to the new worktree and parse all 34 transplanted MATLAB
  files with zero Code Analyzer findings.
- 29 September, migration validation: Pass all 29 SPK, OBJ-selection and mesh
  reader tests with no incomplete checks. Pass panel visibility and four SRP
  source harnesses, including 478 regular derivative queries and 12 pole queries.
  The temporary runner initially omitted the pole harness's required LUT input;
  preserve that failed log and rerun the correctly supplied harness successfully.
  No production correction was needed. Do not claim full simulation, fresh SRP
  codegen, high-degree gravity fitting or broad library qualification from these
  focused checks.
- 29 September, stale cleanup: Verify ancestor coverage, all nine archived files,
  unchanged staged blobs, absence of active processes and absence of ignored
  payloads. Clear only the verified seven staged and two untracked paths, then
  remove `phase-d-kernel-review-staging` through normal Git removal. Use no force
  option. Remove only its obsolete editor entry; retain the new worktree entry.
- 29 September, review: Align SPK argument dimensions and header columns, use
  `self` in both test classes, and clarify folder/source ownership comments.
  Keep test methods free of argument-validation blocks as required by the MATLAB
  test framework. Recheck all ten SPK tests and the six reviewed MATLAB files;
  all pass with zero Code Analyzer findings.
- 29 September, concurrent work: Detect independent staging of SPK files and
  updates to two consolidation documents in the main checkout. Preserve that
  index and those documents. Keep this batch in the new feature worktree and
  leave all donor source in main while the RCS-1 campaign remains active. Do not
  create duplicate commits in both destinations; reconcile the user's eventual
  main-provider commits before integrating this feature branch.
- 29 September, staged handoff: Review the complete nine-file SPK index and
  verify every staged blob matches its working file. Pass cached whitespace
  checks and leave OBJ/SRP candidates, the configuration guideline and editor
  settings outside this batch. The ignored `data/` parent made the first exact
  add return a warning despite staging the tracked manifest; use an update-only
  add for that manifest. Stage no kernels, large data or generated artifacts.
  Leave commits and the next batch to the user.
- 29 September: Only the stale CMake registration was removed. Its branch and
  all six surviving worktrees remain. No source, existing index, branch HEAD,
  dependency pointer or running process changed during that cleanup.
- 29 September: Most kernel source is already copied to main, but its loader
  and test lack the reviewed loaded-file identity check. Main also lacks the
  actuation fix. Copying directories or treating these as redundant would lose
  corrections. The panel-visibility implementation differs in documentation and
  spacing; its test exists only in the review checkout.
- 29 September: Prepare Stage 1 in main. Preserve the original first-order
  random error model and negligible-impulse guard; remove the extra tangential
  scale. Correct the function header, add explicit input/output blocks, document
  every test method and restore the caller RNG after each test. Replace the
  scenario-specific burn values with generic angular-dispersion inputs and add
  deterministic velocity-unit scaling checks.
- 29 September: Both new scaling regressions fail against unchanged HEAD, then
  the full source suite passes 11/11 with the fix. Code Analyzer reports zero
  findings in both reviewed MATLAB files. Fresh fixed-size C++ MEX generation
  passes three edge cases and five angular-dispersion cases. MATLAB exits zero;
  the temporary builder emits a deprecated `DynamicMemoryAllocation` option
  warning. Preserve the initial analyzer-failure log and final validation log in
  the inventory evidence folder. No full simulation was run.
- 29 September: Stage exactly the implementation, its generic test class and
  this plan in main. Inspect the full cached source/test diff; preserve both
  detached review indexes and all unrelated changes. Leave the existing
  Nav-Backend pressure index untouched. Await the user's commit before the
  next batch or any removal of a dirty review checkout.
- 29 September: Verify Nav-Backend's pressure batch is now committed at
  `e6510d315`; its index is empty and the committed patch exactly matches the
  previous reviewed index. No Nav-Backend files or index were modified here.
- 29 September: Verify the first impulse batch is committed at `1f10617`; its
  three-file patch exactly matches the previously reviewed index. Record Stage
  1A as a finite-angle proposal following the user's question. MathCore already
  supplies the active exact rotation with a stable small-angle series and
  codegen support. Leave implementation and the now-empty Git index unchanged.
- 29 September: Implement approved Stage 1A in main. Reuse MathCore's exact
  active rotation and apply the existing signed Gaussian magnitude factor to
  the rotated impulse. Preserve the public arguments, negligible-impulse guard,
  axis projection/fallback and random draw sequence. Change no configuration,
  dependency implementation or pointer.
- 29 September: Prove three finite-angle regressions fail against the committed
  tangent source, then pass all 17 source tests with no incomplete tests. Build
  fresh fixed-size C++ MEX entries for the public function and a temporary
  draw-only probe; pass 30 physical, edge and RNG checks, including the exact
  Gaussian ensemble mean. Keep independent trigonometric expectations in tests;
  introduce no production rotation helper. Code Analyzer reports zero findings.
  MATLAB exits zero. No full simulation was run.
- 29 September: Keep evidence under
  `/tmp/simgears-finite-angle-20260929-u76_nc0j`, including the baseline hashes,
  expected failures and `final_validation.log`. The first verification attempt
  stopped at its source-path assertion because the temporary helper directory
  also contained the archived tangent source. Separate the probe directory from
  the archive; both successful builds resolve the actual reviewed source and
  bundled MathCore. No production correction was needed for that harness issue.
- 29 September: Complete the documentation/readability pass with imperative
  comments, explicit signed-scale/angle semantics and separate sampling/projection
  blocks in the test oracle. Recheck all 17 source tests and Code Analyzer after
  cleanup; MATLAB exits zero. Review and stage exactly the function, test class
  and this plan. Verify staged/worktree byte identity, the exact three-path
  allowlist and cached whitespace. Leave the COSMICA status update unstaged and
  wait for the user's commit.
- 29 September: The final cross-worktree check detects concurrent staging in
  `landing-building-blocks` after the earlier check passed. Leave that index
  untouched; its HEAD is unchanged. The other four worktree indexes, including
  both detached review batches, still match the saved inventory. A later scan
  also detects an independent update to the unstaged kernel-bundle test in main.
  Preserve that file and record unrelated baseline deltas in `final_review.json`;
  keep this extension confined to the three main-checkout paths.
