# Shape Mesh Reader Migration Implementation Plan

> **For agentic workers:** Use the focused test-driven-development and
> verification-before-completion workflows. Track progress with the checkboxes below.

**Goal:** Own validated OBJ/STL geometry loading in SimulationGears, expose it through
`CShapeModel`, and remove the duplicate parser from `space-nav-shape-reconstruction`.

**Architecture:** Add one host-side `LoadShapeMesh` utility with a vectorized triangular-OBJ
fast path and a general validated path. Keep mesh repair explicit. Delegate the existing
geometry-only OBJ API only if parsing with repair disabled remains within 10% of its current
time and peak-memory performance.

**Consumers:** `CShapeModel` and the sibling `space-nav-shape-reconstruction` MATLAB experiment.

## Global Constraints

- Do not modify MathCore, C++, CMake, wrappers, submodules, or repository gitlinks.
- Preserve existing OBJ texture and normal loading behavior.
- Preserve source coordinates and units in the reader; unit conversion remains in `CShapeModel`.
- Keep file I/O host-side without a MATLAB Coder directive.
- During implementation, do not stage, commit, tag, push, or open a pull request without
  separate user authorization for that operation.
- Preserve the existing staged index in `space-nav-shape-reconstruction` unchanged.
- Test observable contracts and explicit option values, not tunable defaults.

## Baseline Evidence

MATLAB R2024b Update 6, warm-cache median timings for the current
`CShapeModel.LoadModelFromObj(..., true)` and the reader being migrated:

| Faces | File size | Current OBJ loader | Reader before optimization | Slowdown |
| ---: | ---: | ---: | ---: | ---: |
| 200 | 5.3 KB | 0.410 ms | 3.191 ms | 7.8x |
| 9,800 | 310 KB | 9.472 ms | 143.285 ms | 15.1x |
| 100,352 | 3.53 MB | 103.207 ms | 1,549.620 ms | 15.0x |

Alternating call order produced 13.6x and 13.9x slowdowns for the two larger cases.
The existing loader also dropped the fourth vertex of a quad, accepted an out-of-range face
index, retained a zero-area face, and rejected STL. These results justify a broader reader but
do not justify unconditional delegation of the optimized legacy path.

## Stage 1 - Shared Reader Contract

- [x] Add focused RED tests for the public `LoadShapeMesh` API and stable output layout.
- [x] Add RED tests for triangular, slash-form, negative-index, convex-polygon, and
  concave-polygon OBJ records.
- [x] Add RED tests for malformed vertices, invalid indices, non-planar/non-simple polygons,
  and empty or unusable geometry.
- [x] Add RED tests for ASCII STL, binary STL, binary `solid` headers, short valid ASCII STL,
  truncated records, and malformed facet structure.
- [x] Add RED tests for explicit repaired and unrepaired output.
- [x] Implement `LoadShapeMesh(charMeshPath, options)` with
  `options.bRepairMesh (1,1) logical = true`.
- [x] Return `ui32FaceVertexIds` as `F-by-3 uint32`, `dVerticesPos` as `N-by-3 double`, the
  source path, and `uint32` vertex/face counts.
- [x] Implement a vectorized fast path for ordinary triangular OBJ geometry.
- [x] Implement deterministic general parsing for variable-length and slash-form OBJ faces.
- [x] Triangulate simple planar convex or concave polygons while preserving source winding.
- [x] Implement robust ASCII/binary STL classification and parsing.
- [x] Validate syntax, finite coordinates, integer indices, bounds, and usable output.
- [x] Weld duplicate vertices, remove degenerate faces, and compact unused vertices only when
  `bRepairMesh=true`.

## Stage 2 - CShapeModel Integration

- [x] Add RED tests for `CShapeModel("file_mesh", ...)` with OBJ and STL inputs.
- [x] Add RED tests for explicit geometry repair and for rejection of repair with auxiliary
  OBJ texture/normal loading.
- [x] Add `"file_mesh"` as a geometry-only constructor loading method.
- [x] Convert shared row-major arrays to the existing internal `3-by-N` layout before unit
  scaling and simplification.
- [x] Extend `CShapeModel.LoadModelFromObj` with `options.bRepairMesh=false` while preserving
  its existing two-argument contract.
- [x] Keep the auxiliary OBJ parser for `bVertFacesOnly=false`.
- [x] Replace the deprecated standalone `LoadModelFromObj.m` parser with a compatibility
  forwarder to the class method.

## Stage 3 - Delegation Performance Gate

- [x] Benchmark the current loader and `LoadShapeMesh(..., bRepairMesh=false)` on identical
  9,800-face and 100,352-face clean triangular OBJ fixtures.
- [x] Warm both implementations and alternate call order before comparing median elapsed time.
- [x] Measure root-function `PeakMem` with `profile on -memory` in three fresh MATLAB sessions.
- [x] Exclude fixture generation, MATLAB startup, welding, degenerate removal, and compaction.
- [x] Record raw samples, medians, ratios, commands, fixtures, and MATLAB version below.
- [x] Delegate default geometry-only OBJ loading only if time and peak memory are each no more
  than `1.10x` legacy at both required sizes.
- [x] If any gate fails, retain the legacy default path and route only `bRepairMesh=true` and
  `"file_mesh"` through `LoadShapeMesh`.

### Gate Results

- Status: **failed on elapsed time; passed on peak memory**
- MATLAB version: R2024b Update 6
- Fixtures: generated positive-index triangular OBJ grids. The 70-by-70-cell fixture has 9,800
  faces, 5,041 vertices, and 310,324 bytes. The 224-by-224-cell fixture has 100,352 faces,
  50,625 vertices, and 3,533,539 bytes.
- Command method: run MATLAB with `-batch`, add SimulationGears through `SetupSimGears`, warm both
  functions, alternate legacy/shared call order, and time only
  `CShapeModel.LoadModelFromObj(path, true)` or
  `LoadShapeMesh(path, bRepairMesh=false)`. Run each memory case in three fresh `-batch` sessions
  with `profile on -memory` and read the root function's `PeakMem`. Temporary fixtures and scripts
  under `/tmp/simgears-mesh-bench.WbNzAS` were removed after results were recorded.
- Timing at 9,800 faces, milliseconds:
  - legacy samples: `[11.593, 11.031, 9.706, 13.032, 11.404, 9.233, 9.127]`
  - shared samples: `[16.279, 16.914, 15.002, 20.521, 14.628, 14.479, 14.162]`
  - medians: `11.031` legacy, `15.002` shared; ratio `1.359985x` (**fail**)
- Timing at 100,352 faces, milliseconds:
  - legacy samples: `[95.420, 103.052, 100.562, 100.008, 98.202]`
  - shared samples: `[152.536, 157.165, 153.760, 152.957, 154.964]`
  - medians: `100.008` legacy, `153.760` shared; ratio `1.537477x` (**fail**)
- Peak memory at 9,800 faces, bytes:
  - legacy samples: `[1049470, 1049220, 1049520]`
  - shared samples: `[887664, 887440, 887664]`
  - medians: `1049470` legacy, `887664` shared; ratio `0.845821x` (**pass**)
- Peak memory at 100,352 faces, bytes:
  - legacy samples: `[9170620, 9170850, 9170750]`
  - shared samples: `[9171010, 9170610, 9171060]`
  - medians: `9170750` legacy, `9171010` shared; ratio `1.000028x` (**pass**)
- Delegation decision: retain the optimized legacy path for default geometry-only OBJ loading.
  Route only explicit `bRepairMesh=true` and `"file_mesh"` through `LoadShapeMesh`.

## Stage 4 - Shape-Reconstruction Migration

- [x] Replace local parser and repair implementation with a thin `LoadShapeMesh` adapter.
- [x] Preserve the package API layout and `ReadShapeMesh:*` error namespace.
- [x] Add the SimulationGears shape-model utility path and `LoadShapeMesh` probe to
  `SetupExperimentPaths`.
- [x] Fail clearly when mesh-backed truth is requested without resolved SimulationGears reuse.
- [x] Keep analytic truth modes independent of SimulationGears.
- [x] Move detailed parser tests to SimulationGears; retain adapter and integration tests in
  shape reconstruction.
- [x] Update examples to enable sibling reuse for mesh-backed truth.
- [x] Mark the reader `extracted` in the migration register and update the semantic-cleanup plan.

## Stage 5 - Verification And Handoff

- [x] Run focused `LoadShapeMesh` and `CShapeModel` tests.
- [x] Run MATLAB Code Analyzer on all modified source and test files.
- [x] Attempt the complete SimulationGears MATLAB suite and record unrelated failures/timeouts.
- [x] Run the complete shape-reconstruction MATLAB suite with available sibling integration.
- [x] Prove analytic shape-reconstruction tests pass without sibling reuse.
- [x] Search both repositories to confirm parsing logic has one authoritative owner.
- [x] Inspect complete diffs, run whitespace checks, and confirm both Git indexes are unchanged.
- [x] Report paths, behavior, benchmark evidence, tests, caveats, and exclusions before the
  staged-review handoff; do not commit without separate authorization.

### Verification Results

- `testLoadShapeMesh`: 9/9 passed.
- New `CShapeModel` integration checks: 5/5 passed, including OBJ/STL `file_mesh`, explicit
  repair, auxiliary-index rejection, and the deprecated forwarder.
- Complete shape-model subsystem: 34/35 passed. The only failure is the pre-existing
  `testDefineShapeModelUsesExplicitApophisElongatedScenario`, whose Blender-path expectation has
  no corresponding `charDefaultBlenderRelativePath` in the ApophisElongated registry entry.
- MATLAB Code Analyzer: `ANALYZER_COUNT=0` across all modified source and test files.
- Complete shape-reconstruction MATLAB suite: passed with sibling integration available.
- Standalone analytic truth check: `smooth`, `localized`, and `elongated_itokawa_like` all passed
  in a fresh process where `which('LoadShapeMesh')` was empty.
- Complete SimulationGears suite attempt: the migration-owned reader and shape-model tests passed
  inside the run, but the repository-wide command encountered multiple unrelated legacy or
  environment failures and timed out after 900 seconds in a CPU-active CR3BP dynamics test.
  Earlier unrelated failures included missing MICE/setup helpers, an intentionally unimplemented
  target-emulator test, the known Apophis Blender-path expectation, and legacy dynamics assertions.
- Ownership search: STL parsing and general polygon triangulation resolve only to `LoadShapeMesh`;
  shape reconstruction contains only its package-layout/error-namespace adapter. The measured
  specialized triangular-OBJ default remains in `CShapeModel` by the explicit performance gate.
- Scoped whitespace, full tracked/cached diff checks, and trailing-whitespace scans passed. The
  SimulationGears index was empty before this reviewed batch was staged. No migration path is
  staged in shape reconstruction; its pre-existing staged infrastructure batch remains separate.
- The implementation phase made no commits. Staging was deferred until explicit user authorization
  for this staged-review handoff.

## Stage 6 - Staged Readability And Simplification Review

- [x] Remove redundant assignments and stale comments in the CShapeModel integration path.
- [x] Separate validation, parsing, transformation, and output steps with logical whitespace.
- [x] Document fast-path fallback, relative-index resolution, polygon triangulation, STL
  classification, grammar state, and repair invariants with purpose-oriented comments.
- [x] Avoid repeated ASCII STL case conversion while preserving accepted syntax and errors.
- [x] Keep the measured default OBJ fast path and binary STL I/O algorithm unchanged.
- [x] Rerun focused reader and CShapeModel integration tests plus MATLAB Code Analyzer.
- [x] Restage the reviewed batch and rerun cached-diff and whitespace gates.

### Readability Review Results

- Removed two self-assignments from `CShapeModel.LoadModelFromObj_`; the established column-major
  geometry layout remains unchanged.
- `ReadAsciiStl_` now case-folds each input record once instead of repeating `lower` calls across
  its state transitions.
- Added comments only at non-obvious ownership, fallback, numerical-tolerance, state-machine, and
  index-remapping boundaries. Straightforward assignments remain self-documenting.
- Added logical spacing between validation, state mutation, geometric transformation, and output
  publication blocks.
- Focused behavior remained green at 9/9 reader tests and 5/5 CShapeModel integration tests;
  MATLAB Code Analyzer reported zero findings across all five staged MATLAB files.
