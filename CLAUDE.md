# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SimulationGears_for_SpaceNav is a MATLAB library for spacecraft navigation simulation. The C++/CMake infrastructure is template scaffolding; the active codebase is under `matlab/`.

The library provides orbital dynamics propagation, sensor/measurement modeling (cameras, LIDAR, ray-tracing), attitude pointing, ephemeris evaluation (SPICE-based), and 3D scene generation for spacecraft proximity operations.

## Setup

Run `SetupSimGears` from any directory (it resolves its own path):

```matlab
run('/path/to/SimulationGears_for_SpaceNav/matlab/SetupSimGears.m')
```

This adds all source directories to the MATLAB path, excluding `.deprecated/`, `codegen/mex/`, and `experimental/`.

## Running Tests

Tests live in `tests/matlab/` mirroring the source tree structure. All new tests must be `matlab.unittest.TestCase` classes.

Run all tests:

```matlab
results = runtests('tests/matlab', 'IncludeSubfolders', true);
```

Run a specific test:

```matlab
results = runtests('tests/matlab/simulation_models/dynamics/testRHS_2BP');
```

## Architecture

### Class Naming Conventions

- `C*` — full-featured handle/value classes (e.g., `CGeneralPropagator`, `CShapeModel`)
- `S*` — lightweight struct-like data classes (e.g., `SPose3`, `SNavState`)
- `Enum*` — enumeration classes (e.g., `EnumFrameName`, `EnumScenarioName`)

### Key Class Hierarchy

```
CBaseDatastruct (abstract, handle)
├── CBaseDatastructWithTimes
├── CCameraIntrinsics
├── CSimulationState
├── CShapeModel → CSceneObject, CTargetEmulator
├── SPose3, SNavState, SReferenceMissionDesign, ...
└── SDatasetFromSimStateIntermediateRepr

CGeneralPropagator (handle)
└── CScenarioGenerator
```

**CBaseDatastruct** (`matlab/general_utils/datastructs/CBaseDatastruct.m`) — foundation for all data classes. Provides serialization (`toStruct`, `toYaml`, `toJson`, `toMatFile`) and deserialization (`fromStruct`, `fromYaml`, `fromMatFile`), automatic data hashing, and field validation. All new data classes should inherit from this.

**CGeneralPropagator** (`matlab/simulation_models/propagators/CGeneralPropagator.m`) — core trajectory propagation engine. Holds initial state, time grids, ODE solutions, and dynamics function handles. Entry point: `propagateOrbitTrajectory()`.

**CScenarioGenerator** (`matlab/simulation_models/generators/CScenarioGenerator.m`) — top-level orchestrator extending `CGeneralPropagator`. Assembles orbit dynamics, attitude, ephemerides, sensor config, and acceleration data into a complete simulation scenario.

### Module Layout

| Directory | Purpose |
|---|---|
| `matlab/simulation_models/dynamics/` | ODE right-hand sides (`RHS_*` functions): 2BP, CR3BP, N-body, Gauss, STM variants |
| `matlab/simulation_models/accelerations/` | Force models: spherical harmonics, SRP, J2, aerodrag |
| `matlab/simulation_models/measurements/cameras/` | Pinhole projection, landmark visibility, ray-tracing |
| `matlab/simulation_models/measurements/common/` | Ray-geometry intersections (Moller-Trumbore algorithm) |
| `matlab/simulation_models/measurements/lidar/` | Laser rangefinder model and noise |
| `matlab/simulation_models/propagators/` | Propagator classes including CasADi-based variant |
| `matlab/simulation_models/generators/` | Scenario generation and environment builders |
| `matlab/simulation_models/shape_models/` | Triangular mesh representation, OBJ import/export |
| `matlab/simulation_models/attitude/` | Attitude dynamics and pointing generation |
| `matlab/general_utils/datastructs/` | Base data classes and serialization infrastructure |
| `matlab/simulation_management/` | Config, SPICE kernel loading, mode management |
| `matlab/codegen_scripts/` | MATLAB Coder scripts for MEX generation |

### MEX Code Generation

Performance-critical ray-tracing functions have MEX-compiled versions under `matlab/simulation_models/sensors/cameras/codegen_scripts/codegen/mex/`. Codegen scripts in `matlab/codegen_scripts/` produce these. Pre-compiled binaries target Linux x86-64 (`.glnxa64`).

### Variable Naming Conventions (Hungarian prefix)

All variables use type prefixes — never use bare single-letter names:

- `d` -- double (e.g., `dMassRatio`, `dPosVel`, `dRadius`)
- `b` -- logical (e.g., `bHasData`, `bIntersectFlag`)
- `ui32` / `ui16` / `ui8` -- unsigned integer (e.g., `ui32MaxDegree`)
- `str` -- struct (e.g., `strDynParams`)
- `i_` / `o_` -- input/output prefix in some functions
- `id` -- loop index (e.g., `idT` for time, `idP` for point)

### Function Header Format

All modernized functions use this header structure:

```matlab
%% PROTOTYPE
%% DESCRIPTION
%% INPUT
%% OUTPUT
%% CHANGELOG
%% DEPENDENCIES
```

With `arguments (Input)` and `arguments (Output)` blocks for validation.

### Naming Conventions for Functions

- `RHS_*` -- right-hand side of ODE systems (dynamics)
- `compute*` / `Compute*` -- computation functions
- `generate*` / `Generate*` -- data generation functions
- `RayTrace*` / `Ray*` -- ray-tracing and intersection functions

### External Dependencies

- **SPICE Toolkit** (mice/cspice) — ephemeris and kernel operations
- **CasADi** (optional) — symbolic automatic differentiation in `CCasadiPropagator`
- **SnakeYAML** (optional) — YAML serialization in `CBaseDatastruct`
- **MathCore_for_SpaceNav** — math utility submodule in `lib/`
- Target MATLAB release: **R2023b**
