# CBaseDatastruct Serialization Hardening Plan

## Summary

Harden `CBaseDatastruct` export behavior in the active COSMICA development dependency:
`/home/peterc/devDir/SimulationGears_for_SpaceNav`.

The goal is to prevent YAML/struct export from recursively serializing arbitrary MATLAB runtime objects such as
neural-network models, while preserving export of structs, primitives, enums, cells, and objects derived from
`CBaseDatastruct`.

## Evidence

- COSMICA development setup resolves `CBaseDatastruct` to
  `/home/peterc/devDir/SimulationGears_for_SpaceNav/matlab/general_utils/datastructs/CBaseDatastruct.m`.
- The same setup resolves `DefineShapeModel` to the same devDir SimulationGears checkout.
- Current `convertValue_` serializes arbitrary objects by calling any available `toStruct` method, or by falling
  back to `struct(inVal)`.
- A probe confirmed a non-`CBaseDatastruct` `timer` object becomes a struct.
- A probe confirmed table YAML export fails with `MATLAB:table:LinearSubscript`.
- The COSMICA failure is consistent with `objScenarioConfig.objSensorParams.objModelNCOB` being expanded into
  neural-network internals containing `Connections`, `Learnables`, and `State` tables.
- Existing `testCBaseDatastructExport.m` had no coverage for non-base objects, tables, timetables, or containers.

## Key Changes

- Modify `CBaseDatastruct.convertValue_` so only `CBaseDatastruct` instances are recursively exported as objects.
- Preserve enum conversion to strings.
- Preserve recursion through structs and cells.
- Treat non-`CBaseDatastruct` objects, tables, timetables, and `containers.Map` as non-serializable runtime payloads
  and omit them from exported structs/YAML.
- Tighten root validation so top-level export input is only a struct or `CBaseDatastruct`; arbitrary top-level objects
  are rejected.
- Do not add special neural-network logic. The fix should be generic and small.

## Implementation Steps

- Add focused tests in `tests/matlab/general_utils/datastructs/testCBaseDatastructExport.m`.
- Add a helper class under `tests/matlab/test_helpers`, for example `CNonBaseSerializationProbe`, that does not
  inherit from `CBaseDatastruct` and includes scalar fields plus a table field.
- Test that a nested non-base object is omitted by `toStructStatic`.
- Test that nested table, timetable, and container fields are omitted before YAML export.
- Test that nested `CBaseDatastruct` values still serialize normally.
- Test that enums still serialize to strings.
- Test that top-level arbitrary objects are rejected by `toStructStatic` or `toYamlStatic`.

## Test Plan

- Focused serializer export tests:
  `matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath(genpath('matlab')); addpath(genpath('tests/matlab')); results = runtests('tests/matlab/general_utils/datastructs/testCBaseDatastructExport.m'); disp(results); assert(all([results.Passed]));"`
- Import tests if MATLAB startup is stable:
  `matlab -batch "cd('/home/peterc/devDir/SimulationGears_for_SpaceNav'); addpath(genpath('matlab')); addpath(genpath('tests/matlab')); results = runtests('tests/matlab/general_utils/datastructs/testCBaseDatastructImport.m'); disp(results); assert(all([results.Passed]));"`
- Isolated COSMICA failing module from the saved output pack; verify `coreConfigPack` YAML export no longer fails.
- Do not rerun the full COSMICA simulation for this fix.

## Assumptions

- The active source of truth for this fix is devDir `SimulationGears_for_SpaceNav`, not the nav-backend embedded copy.
- Syncing the embedded nav-backend SimulationGears submodule is a separate follow-up task.
- Omitting arbitrary runtime objects from YAML/struct export is preferred over attempting lossy serialization of their
  internals.
