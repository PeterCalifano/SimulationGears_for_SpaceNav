# B00 build baseline and consolidation review

27 September 2026. Scope: existing library infrastructure. No physical model or
landing qualification is claimed by these checks.

## Revisions and environment

| Item | Selection |
| --- | --- |
| SimulationGears | `e86d14d8bbab73ef79ac60f1b2c870c83072706e` |
| EstimationGears | `991ebbc78f8645b4331cab56c39c3e848335777b` |
| MathCore | `e3a39c68b8a9445a39c783a64b881f1bdf302b0e`, unchanged |
| Clean external wrap | `6f90752cfcfe629ad8b3f05af6676de4ef4c69b0`, unchanged |
| Compiler / CMake / Python | GCC 13.3.0 / CMake 3.28.3 / Python 3.12.3 |
| MATLAB | R2024b, 24.2.0.2923080 |
| CUDA | Disabled |

Both new `feature/landing-building-blocks` worktrees started with empty indices.
Original dirty worktrees and running experiments were preserved.

## Results

- SimulationGears unmodified native baseline builds.
- Its existing asset-fetch pytest suite has 6 passes and 2 pre-existing failures:
  Apophis/Toutatis manifests use `obj`, while the parser expects `wavefront_obj`.
  This unrelated issue is recorded without changing the manifests.
- EstimationGears native and existing MATLAB/Python wrappers build. All 18 existing
  CTest cases pass. Installed C++, Python and MATLAB consumers pass; MATLAB exits
  normally. No EstimationGears source change was needed.

Evidence: `/tmp/cosmica-landing-b00-20260927/`, particularly `sg-baseline-*`,
`eg-baseline-*`, `eg-installed-matlab.log` and `eg-consumer*`.

## User correction and implementation policy

The initial SG logger-only interface, wrapper probes, consumer project and CMake
edits were removed following the user's correction. Their generated build artifacts
and logs remain historical evidence only; they do not describe current source.
Do not reuse that install as evidence for subsequent physical model changes.

Use existing source/test discovery and `wrap` generation for real model APIs.
Add no CMake tests and make no speculative build or wrapper-framework edits.
If a concrete failure occurs, check interface usage first, then compare the
relevant files against `cpp_cuda_template_project`. The user authorized a justified
template update when stale infrastructure is the cause. Record the donor revision,
affected contract and verification before importing such an update.

At the original B00 inspection, the template checkout was `c9b191f`. That inspection
established working model builds, not alignment with every template export contract.
The following review supersedes its template-alignment status.

## Historical prerequisite audit: verify template v2.0.3 alignment

Reviewed 29 September 2026 before the source consolidation below. These probes
used the original tracked revisions; their failures describe that baseline.

### Scope and revisions

| Checkout | Branch / revision | Review scope |
| --- | --- | --- |
| Template donor | `main`, tag `v2.0.3`, `b7dc26c4a3fdd90eb1b5974f3f796cff1e565438` | Read the exact release and inherited `v2.0.0..v2.0.3` build changes |
| SimulationGears landing worktree | `feature/landing-building-blocks`, `e86d14d8bbab73ef79ac60f1b2c870c83072706e` | Primary consolidation owner; model changes remain unstaged |
| SimulationGears original checkout | `1f10617d2d9bc7599bfe0e6e41e9dccb28fe9816` | Compare build files only; no newer fix for the findings below is present there |
| EstimationGears landing worktree | `feature/landing-building-blocks`, `991ebbc78f8645b4331cab56c39c3e848335777b` | Inspect build/consumer prerequisites; preserve its clean tree |
| COSMICA | `develop`, `128fc97959749ac020410feaed385dfd86c08f34` | Check build presence and update landing coordination notes only |

Both landing indices were empty at entry. The donor was clean. An HTTPS
`git ls-remote` confirmed that remote `main` and the peeled `v2.0.3` tag resolve
to the same donor commit; the initial SSH lookup failed with a connection reset.
No branch, tag, gitlink or external wrapper checkout was changed.

### Findings and tailoring decisions

| Area | Evidence / decision |
| --- | --- |
| Shared CMake helpers | SG: 31 exact matches and five differences limited to trailing whitespace. EG: 33 exact matches and one difference limited to comments/whitespace. Retain these helpers; no functional replacement is justified |
| Build entry point and wrappers | `build_lib.sh`, `HandleWrapper.cmake`, `HandleMatlabWrapper.cmake`, `HandlePythonWrapper.cmake` and `StagePythonRuntimeArtifacts.cmake` match the donor exactly in both libraries. Preserve the existing `wrap` route |
| Inherited fixes | Source archive naming/exclusions, single trailing newline in VERSION output, prefix-relative MATLAB installation and the installed-gtwrap header bridge are already present. Do not reimplement them |
| **Required: public header contract** | Both libraries still export raw `src` include roots. Package-qualified includes fail for build-tree consumers, while installed consumers accept both spellings. Adopt the donor package-prefix build view and generated-config path, keep source-relative includes private, and migrate actual consumer/interface includes together |
| **Required: SG Python version metadata** | `python/pyproject.toml.in` substitutes `PROJECT_VERSION`, bypassing the existing helper's `PYTHON_PACKAGE_VERSION`. A synthetic `1.2.3-rc.1` probe produces `1.2.3` instead of `1.2.3rc1`. Use the donor variable without changing the versioning helper. EG already uses it |
| Product identity and dependencies | Preserve SG/EG names, namespaces and versions; Eigen, SG Threads, current source modules, optional features and project presets. Keep SG's `SANITIZE_BUILD` condition and C++20 propagation rather than copying the donor root wholesale |
| Intentional omissions | Do not add TensorRT, ZeroMQ, donor examples, placeholder APIs or donor CMake regression suites to SG. EG intentionally omits OptiX/PTX and automatic native submodule composition; preserve those choices |
| Optional OptiX export | SG does not include the donor package's OptiX SDK rediscovery block. Its current source inventory has no PTX kernel and already rejects enabling OptiX. Revisit this dormant export path only if SG gains an actual supported OptiX target; renderer ownership remains external |
| Python package deployment | Existing model wrapper execution does not certify a relocatable wheel. Inspect SG's currently empty runtime-target declaration and exercise a relocated package during the bindings stage; do not change the shared packager speculatively |
| Future COSMICA native work | This inspected COSMICA checkout has no root CMake project/build helper. The existing upgrade prerequisite remains open before adding native implementation there; do not import a framework merely to store the current MATLAB orchestration/design notes |

The public-header mismatch is an inherited export-contract gap, not a failed
physics model or a need to rewrite `wrap`. The Python version finding concerns
`pyproject.toml.in` versus the helper's computed version; `setup.py.in` delegates
version metadata and does not define a competing version.

### Fresh validation

Export each library's tracked `HEAD` into a separate audit source directory with
`git archive`; configure/build/install it with the existing CMake files. Disable
CUDA, OpenGL, wrappers, tests, examples and programs for these bounded native
consumer probes. Disable SG's optional submodule composition. Use GCC 13.3.0,
CMake 3.28.3, Ninja, Release and `CPU_ENABLE_NATIVE_TUNING=OFF`.

Each disposable consumer uses only `find_package(... CONFIG REQUIRED)`, the
exported library target and C++20; it supplies no manual header search paths.
SG calls the existing logger; EG calls the existing wrapper-placeholder native
symbol. No landing-model source from the untracked working tree enters these
builds. These probes are acceptance commands, not new registered CMake tests.

| Check | SG | EG |
| --- | --- | --- |
| Fresh native configure / build / install | Pass | Pass |
| Build-tree consumer, package-qualified header | **Fail: header not found** | **Fail: header not found** |
| Build-tree consumer, bare header; compile/link/run | Pass | Pass |
| Installed consumer, package-qualified header; compile/link/run | Pass | Pass |
| Installed consumer, bare header; compile/link/run | Pass, contrary to donor's strict prefix contract | Pass, contrary to donor's strict prefix contract |
| Synthetic prerelease Python metadata | **Fail: loses `rc1`** | Not rerun; template uses the donor variable |

Failing includes:

```cpp
#include <SimulationGears_for_SpaceNav/utils/logging/CLogger.h>
#include <EstimationGears_for_SpaceNav/wrapped_impl/CWrapperPlaceholder.h>
```

Local evidence is under `build/landing/review-template-v2.0.3/`:

- `commands.json`: exact native configure/build/install/consumer commands and exits
- `helper-comparison.json`: per-file donor comparisons
- `sg-*` / `eg-*` logs and disposable source/build/install/consumer directories
- `version-probe/configure.cmake` and `result.json`: executable synthetic version check

For example, reproduce the SG build-tree failure after its native build with:

```sh
cmake -S build/landing/review-template-v2.0.3/sg-build-qualified \
      -B build/landing/review-template-v2.0.3/sg-build-qualified/build \
      -DCMAKE_PREFIX_PATH="$PWD/build/landing/review-template-v2.0.3/sg-build"
cmake --build build/landing/review-template-v2.0.3/sg-build-qualified/build
cmake -P build/landing/review-template-v2.0.3/version-probe/configure.cmake
```

The audit did not rerun CUDA, ROS, physical-model tests, wrapper generation,
MATLAB or wheel relocation. Prior B00/B03/B04 results retain their original scope
and dates. They do not resolve these newly exercised consumer-contract failures.

## Review batch 1 — Rename the package and align public exports

29 September 2026. Replace the rejected 86-file batch with the bounded export repair
required by the reproduced v2.0.3 failures. Extend this batch with the subsequently
requested `sim-gears-for-space-nav` package rename. Preserve every native implementation,
test and example in the working tree for later component reviews. The user now
authorizes the export/name commit and continuation to the next component batch.
Keep the two unrelated MATLAB changes outside all landing batches.

### Staged scope and rationale

| Staged path | Required change |
| --- | --- |
| `CMakeLists.txt` | Expose package-qualified headers and generated config; define native and wrapper-safe identities and package artifact names |
| `src/CMakeLists.txt`, existing attitude/logging module CMake files and package config template | Generate qualified config, install under the new prefix and rename native exports/output. Leave landing module additions unstaged |
| `python/pyproject.toml.in`, renamed package directory | Use the existing PEP 440 version; name the distribution with hyphens and the import with underscores |
| `doc/Doxyfile.in` | Name the public package and exclude symbolic links so build-tree scanning does not exclude the source tree |
| Presets and two native CI workflows | Update only project-qualified option spellings |
| ROS shim/bridge CMake files | Forward renamed core options and consume the renamed native package/target |
| README, main API page, ROS and release guides | Document package usage, wrapper identifiers and migration |
| `ros2/simulation_gears_ros/src/conversions.cpp` | Use the exported config-header spelling |
| `ros2/simulation_gears_ros/test/test_build_info_conversions.cpp` | Apply that same public-header spelling in the existing consumer test |
| This report (through the batch sequence below) | Record the reproduced failures, exact repair, independent acceptance and next component boundaries |

Shared CMake helpers, wrap generators, native model sources, module/test registration,
formatting policy and landing API documentation remain outside this batch. The ROS
include migration is required by the changed public export; no ROS behavior changes.

### Initial export acceptance before the rename

Materialize the Git index under `/tmp/sg-export-stage-20260929-*/source` with
`git checkout-index`, then configure and build that isolated snapshot. It contains
only the tracked baseline and this batch; later untracked model code cannot satisfy
missing dependencies. Keep CUDA, OpenGL, submodule composition, runtime tests and
wrappers off for this export probe. Enable HTML/XML and documentation warnings as
errors. Use GCC 13.3.0, Release, CPU native tuning off and Unix Makefiles.

Disposable C++ consumers call the existing logger and include the generated config
header, linking only the exported CMake target. Check build-tree and relocated
installation use, plus rejection of bare header spellings. Reproduce the synthetic
Python prerelease metadata separately through the existing helper/template.

- [x] Build and install the index-only native snapshot.
- [x] Compile, link and run qualified build-tree and relocated consumers.
- [x] Reject bare public headers in both consumer configurations.
- [x] Build HTML/XML and confirm existing native API content is actually present.
- [x] Confirm `1.2.3-rc.1` produces `1.2.3rc1` in Python metadata.
- [x] Inspect the complete seven-file cached diff and its whitespace check.

The first snapshot was placed below `build/`; the documented `*/build*/*` Doxygen
exclusion correctly excluded it. Move the disposable snapshot outside that pattern
and rerun successfully without changing product configuration. Preserve the failed
attempt in the logs. The Git-free snapshot also emits the existing missing-version
fallback warning; the explicit prerelease probe verifies the changed metadata path.

Evidence: `build/landing/review-batch-01/commands.json` and adjacent logs. This initial probe
provides no model, wrapper, ROS or CUDA execution evidence; retain their separately
dated results under the later component review. Do not rerun a full simulation for
these export-only changes.

### Package rename review and acceptance

29 September 2026, baseline HEAD `2b4ff4eb`. Use `sim-gears-for-space-nav` for
`find_package`, exported namespace/target, public header prefix, native library
filename, CPack artifacts and Python distribution metadata. Use
`sim_gears_for_space_nav` for the CMake project/internal target/options and Python/
MATLAB wrapper module identifiers. Keep the established C++ and MATLAB class
namespace `simulation_gears`. The unchanged v2.0.3 helpers derive wrapper names
from the project/target; hyphens cannot appear in generated Python/MATLAB identifiers.
A separate native identity follows the existing gtsam-space-nav library pattern
without adding generator branches or modifying shared helpers.

The staged scope is 20 files, including two renames and this report. Every added
path carries a direct package-name consumer or definition. Landing source includes,
future module install destinations, wrapper imports and the fixture guide are also
updated in the working tree, and remain attached to their later component batches.
Repository paths, remote URLs, historical reports and ROS package identities retain
their established spellings. Configure a fresh build and update consuming includes,
CMake names/options and imports; no old package-name compatibility alias is provided.

- [x] Build/install the index-only snapshot and compile/link/run real C++ consumers
  against both build-tree and relocated exported packages.
- [x] Build HTML/XML for both index-only and full working trees with warnings as
  errors. Present the hyphenated CMake target in a fenced CMake example to avoid
  Doxygen interpreting it as a C++ symbol link.
- [x] Build the full working native library and both wrappers from a fresh cache.
  Pass all 51 CTests (50 native, one Python asset suite), 12 Python model tests and
  the MATLAB numerical/legacy-burn harness, with MATLAB exit code zero.
- [x] Copy the full install to a separate temporary prefix, remove inherited loader
  paths, verify the new distribution/import metadata and pass all 12 Python model
  tests using its packaged shared library without build-link metadata.
- [x] Build the index-only ROS shim, interfaces and bridge in an isolated colcon
  workspace; pass its existing build-info conversion CTest.
- [x] Inspect the complete cached diff and whitespace; retain all model additions
  and unrelated MATLAB edits outside the index. Commit only with explicit authorization.

Evidence: `build/landing/package-rename/`, including commands, staged snapshot path,
full build/wrapper logs and relocated consumers. The baseline index has no native
unit-test targets: an initial CTest probe correctly reported no tests, and the staged
package is qualified by real consumers and its existing ROS conversion test. Full
working-tree tests remain separate evidence. Earlier asset-manifest exclusions no
longer apply at the newer baseline HEAD. GCC Eigen inlining warnings and colcon
unused-option/version-fallback warnings remain; do not claim warning-free compilation.
CUDA, ROS launch behavior, wheel archives and full landing qualification were not run.

Authorized subject: **Align public package exports with template v2.0.3 and rename library**

Proposed body:

- Rename native exports and distribution metadata; keep wrapper identifiers valid

- Expose qualified headers consistently from build and installation trees

- Preserve prerelease Python versions and native API documentation

- Update config-header consumers and verify the isolated staged tree

### Component batch sequence

Each component batch must include its dependent native tests, applicable wrapper
API/tests and necessary documentation. Stage shared files by hunk so a batch never
refers to a later component. Verify each candidate from an index-only snapshot.
Retain the current full-tree qualification as additional evidence, not a substitute.

- [x] **1. Public exports.** The 20-file export/name scope above; staged and independently
  verified and approved for commit with the title above.
- [ ] **2. Spacecraft engineering profiles.** Provenance, mass properties, mounts,
  engine/wheel parameters and enum/custom registry; profile tests and fixture guide.
  Introduce the reference formatting policy with this first native component batch.
- [ ] **3. Stochastic and clock primitives.** Random streams, scalar processes and
  minimum bias/drift clock; independent statistics, replay and clock checks.
- [ ] **4. Finite thrust and prescribed pointing.** Integration boundaries, engine
  histories, attitude references and prescribed-attitude translation; rocket/event
  tests and finite-burn examples.
- [ ] **5. Fixed-mass mechanics and attitude actuation.** General concepts/RK4,
  rigid-body/wheel mechanics and wheel/RCS realization/allocation; independent
  rotation, actuator, oscillator and fixed-mass control checks.
- [ ] **6. Evolving-mass dynamics and event propagation.** Mass policies, reserve/
  wheel events, combined wrappers and powered benchmark; independent variable-mass
  references, rejected flux configurations and combined qualification report.
- [ ] **7. EstimationGears prerequisite.** Review its export repair in that owner's
  index before new estimator implementation.
- [ ] **8. COSMICA handoff.** Review the full landing plan and owning upgrade-index
  changes in COSMICA, with scientific blockers and future consumer gates explicit.

Stage one batch, review its complete index and stop. Advance on `next` after the
preceding index is clear. Commit only with separate explicit authorization.
