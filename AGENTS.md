# Repository Guidelines

Write to `CONTEXT.md` before compaction to prevent context loss. After automatic
compaction, read `AGENTS.md` and `CONTEXT.md` before resuming work.

## Project Structure and Ownership

SimulationGears_for_SpaceNav is primarily a MATLAB library for spacecraft
navigation simulation. Active MATLAB code lives under `matlab/`, with tests
under `tests/matlab/` and development plans under `doc/developments/`. MEX
builder entry points live under `matlab/builders/mex/`; generated MEX artifacts
belong under `matlab/mex/` and must not pollute the repository root.

The root CMake, C++, CUDA, Python, wrapper, and devcontainer infrastructure is
derived from `cpp_cuda_template_project`. Keep general build infrastructure in
sync with that source where practical, while preserving SimulationGears-specific
MATLAB behavior. `CLAUDE.md` provides repository navigation and architecture
context; this file owns normative development and review policy.

Nested libraries and sibling repositories have separate ownership. Identify the
owning repository before editing a dependency, and do not update submodule
pointers or sibling checkouts unless the task explicitly includes that work.

## Build, Test, and Development Commands

- `run('matlab/SetupSimGears.m')`: add supported SimulationGears MATLAB sources
  to the MATLAB path.
- `matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); run('matlab/SetupSimGears.m'); addpath(genpath('tests/matlab')); results = runtests('tests/matlab', 'IncludeSubfolders', true); assert(all([results.Passed]));"`:
  run the MATLAB test suite.
- `matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); run('matlab/SetupSimGears.m'); results = runtests('tests/matlab/simulation_models/dynamics/testEvalRHS_InertialDynMaxFidelity.m'); assert(all([results.Passed]));"`:
  run a focused MATLAB regression.
- `./build_lib.sh`: configure, build, and test the C++-first library surface.
- `./build_lib.sh -p -m`: enable Python and MATLAB wrapper builds.
- `git diff --check` and `git diff --cached --check`: check working-tree and
  staged whitespace before review.

Prefer focused tests for the changed subsystem before broader suites. Report
MATLAB assertion results separately from wrapper teardown warnings or crashes.

## Optional ROS 2 Overlay

The canonical template policy is retained here even when the overlay artifacts
have not yet been imported into this checkout. Adding `doc/ros2_overlay.md`,
`build_ros2.sh`, `ros2/`, or related workflows is a separate template-sync task.

See `doc/ros2_overlay.md` before changing the optional ROS 2 overlay.
`./build_lib.sh` is the C++-first library entry point and never needs ROS.
`./build_ros2.sh` is the optional ROS 2 overlay build and test entry point.

Keep ROS-related changes confined to `ros2/` plus the documented root helpers,
docs, tests, markers, and the single ROS overlay workflow.

## Canonical Language Conventions

### Python

Use Python 3.12 or newer. Matplotlib is the default plotting backend; use PIL or
OpenCV for image work and prefer seaborn for statistical plots. Use PyTorch for
machine-learning applications, supported by scikit-learn where appropriate.

Function names begin with a capital letter and use snake case; class methods
begin with a lower-case letter. Internal non-public methods start with `_`, and
local variables end with `_`. Prefer dataclasses over dictionaries and enums
over literals when there are more than two choices. Type hints are mandatory.
ONNX export compatibility is generally required. New classes and functions
must include a runnable example and expected output.

### C++ and CUDA

C++17 and C++20 are the core standards. CUDA 12.6 or newer is preferred. Prefer
concepts over SFINAE, Catch2 for unit tests, and classes over structs. Inspect
nearby files before choosing names or layout. Keep explanations technical and
focused while making the relevant concepts clear.

### MATLAB

Use classes when stateful behavior benefits from them; prefer functions for
stateless algorithms. Use `self` instead of `obj` for class method receivers.
Use explanatory Hungarian-style type prefixes because MATLAB variables are not
statically typed:

- `d` for double and `f` for float.
- `b` for logical values.
- `str` for structs, not strings.
- `char` for strings and character arrays.
- `ui8`, `ui16`, `ui32`, and analogous prefixes for unsigned integers.
- `i8`, `i16`, `i32`, and analogous prefixes for signed integers.
- `obj` for objects, `cell` for cell arrays, `table` for tables, and `bus_` for
  Simulink buses.

Variable names use Pascal case including the prefix, for example
`ui8MyVariable`. Names must explain purpose; short names are allowed only in a
very local scope and should preferably use `Tmp`. Function names and static
methods begin with a capital letter. Local functions end with `_` to identify
private file-local behavior.

Do not nest function definitions. Put a non-reused helper after the primary
function in the same file; move reusable behavior to its own file. Algorithmic
MATLAB code should remain code-generation safe where practical, especially
when a codegen directive is present. Code-generation names must remain within
31 characters.

Use `arguments` and `arguments (Output)` blocks for public inputs and outputs
where supported by the existing API. Follow this documentation structure:

```matlab
function tableValidObservations = LoadValidObservations(charInputPath)
%% SIGNATURE
% tableValidObservations = LoadValidObservations(charInputPath)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Load and validate observations while preserving their input order.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% charInputPath             Path to the delimited input file.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% tableValidObservations    Valid observations in input order.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% DD-MM-YYYY  Pietro Califano     First prototype.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ParseObservations
% -------------------------------------------------------------------------------------------------------------

arguments
    charInputPath (1, :) char
end

arguments (Output)
    tableValidObservations table
end

% Parse all rows through one path so malformed input produces consistent
% diagnostics.
tableParsedObservations = ParseObservations(charInputPath);

% Enforce the domain validity contract without changing source ordering.
bValidObservation = tableParsedObservations.bIsValid;
tableValidObservations = tableParsedObservations(bValidObservation, :);

end
```

## Staged-Code Review Quality Gate

Before handing staged changes to the user for commit review, inspect the
complete Git index with `git diff --cached`. Apply this gate to files staged by
either the user or the agent. This review does not authorize staging,
committing, or rewriting unrelated code.

For every staged source file that is new or substantially modified:

- Add or update both levels of applicable documentation: the file/module-level
  header and the public class/function/method documentation. Follow established
  files for the relevant language and component.
- Organize related statements into visually separated blocks. Each block must
  implement one immediate objective or implementation step.
- Introduce each non-obvious block with a concise comment explaining what it
  accomplishes and, when relevant, why that approach is required.
- Prefer purpose-, invariant-, and contract-oriented comments. Do not add
  comments that merely translate individual statements into prose.
- Preserve useful existing comments and documentation unless the staged change
  makes them incorrect.
- Review the staged result as a reader will receive it, not only the individual
  lines edited during implementation.

Limit cleanup to the intended staged scope. Do not rewrite unrelated legacy
code merely because the same file is staged. Do not report changes as ready for
review until this pass is complete; summarize documentation or readability
cleanup performed during the pass.

### C++ and CUDA Documentation Pattern

Use Doxygen for file headers and public API documentation:

```cpp
/// @file observation_loader.cpp
/// @brief Loads validated observations from a delimited input file.
/// @details Owns parsing and validation; filtering policy remains with the
///          caller.

/// @brief Load and validate observations from disk.
/// @param inputPath Path to the delimited input file.
/// @return Valid observations in input order.
/// @throws std::runtime_error When the input cannot be parsed.
std::vector<CObservation> LoadValidObservations(
    const std::filesystem::path& inputPath);
```

### Python Documentation Pattern

Use Google-style module, class, method, and function docstrings. Keep type hints
on every callable and follow the repository naming conventions:

```python
"""Load and validate observation records."""


def Load_valid_observations(input_path_: Path) -> list[Observation]:
    """Load valid observations while preserving their input order.

    Args:
        input_path_: Path to the delimited input file.

    Returns:
        Valid observations in input order.

    Raises:
        ValueError: If an input row cannot be parsed.
    """
```

## Functional Staging Policy

Inspect `git status --short` before editing. Group staging by coherent behavior,
not merely by file type. A batch should contain its implementation, focused
tests, and directly corresponding development-plan update. Keep cross-repository
consumer migrations in their owning repository and in a separate batch from the
shared-library contract they consume.

Before adding a new batch, review the complete currently staged diff. Preserve
pre-existing staged and unstaged changes; do not unstage, rewrite, or absorb
unrelated work. Do not create commits unless the user explicitly requests them.

## Agent-Specific Instructions

Use `rg` or `rg --files` for repository searches. Use `apply_patch` for manual
file edits. Inspect repository ownership and dirty state before changes. Keep
development plans under `doc/developments/` synchronized with implemented
status, but do not mark work complete without verification evidence.

For MATLAB dynamics changes, preserve code-generation-safe fixed struct shapes
and field order where downstream MEX builds depend on them. Test model-selection
contracts using payloads that contain all relevant data so availability cannot
be confused with selection.
