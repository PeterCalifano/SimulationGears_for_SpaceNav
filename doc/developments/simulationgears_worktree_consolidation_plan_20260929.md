# SimulationGears worktree consolidation

29 September 2026. Consolidate reviewed changes into the existing
`feature/upgrade-filter-smoother-interface` checkout. Review, test and stage one
functional batch at a time; leave commits to the user. Preserve existing indexes,
active work, dependency pointers and external assets. Create no additional worktrees.

## Verified inventory

Use `/home/peterc/devDir/SimulationGears_for_SpaceNav` as the destination. Its
current HEAD is `6c9db696`; its index was empty at inspection. Relative commit
counts below compare each worktree with that HEAD.

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

Remote freshness is unverified: `git fetch origin --prune` failed with an SSH
connection reset. All commit comparisons here use local history and existing
remote-tracking refs.

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
- [ ] Refresh remote comparisons when the connection is available.

## Stage 1: Consolidate impulse execution errors

- [x] Review the isolated two-file correction against main, preserve its
  small-angle model and remove the extra impulse-magnitude multiplication.
- [x] Replace the Phase-D-specific regression with a generic angular-dispersion
  check across impulse magnitudes. Preserve the existing behavioral tests.
- [x] Review function/class documentation, imperative comments, naming and
  formatting. Run the focused suite, Code Analyzer and whitespace checks.
- [x] Stage only the reviewed implementation, tests and this plan in main.
  Inspect the complete index and suggest an imperative commit message.
- [ ] Wait for the user's commit; verify the committed source preserves the
  isolated correction before considering worktree removal.

## Stage 2: Consolidate selected kernel bundles and segmented states

- [ ] Review the seven-file staged kernel batch against the source already in
  main. Carry the loaded-path multiset validation and its same-count wrong-set
  regression; preserve existing segmented-state and manoeuvre extraction code.
- [ ] Keep bundle identifiers and asset metadata in manifests. Keep utilities
  independent of Apophis/RCS-1 and leave SPK/large asset payloads external.
- [ ] Run bundle and segmented-extraction suites with real MICE setup; report
  missing-asset skips separately from passes. Review and stage the coherent batch.
- [ ] Wait for the user's commit and verify both staged implementations survive
  in the destination before closing this review checkout.

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
  Never use force removal to bypass an unreviewed dirty checkout.
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

## Progress and discrepancies

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
