# Native logging

`simulation_gears::logging::CLogger` is the root native logging facility. It is
implemented in `src/utils/logging/` without an external logging dependency.

## Levels and routing

`ELogLevel` is ordered from `Quiet` through `Critical`, `Error`, `Warning`,
`Info`, `Debug`, and `Trace`. A logger emits a message when its configured level
includes that severity. The default level is `Error`.

Warnings, errors, and critical messages use the diagnostic stream. Info, debug,
and trace messages use the output stream. Callers may inject both streams for
tests or integration; otherwise the normal process streams are used. Complete
lines are serialized across logger instances so concurrent messages do not
interleave.

`ELogColorMode` explicitly enables or disables ANSI colors, and every colored
line emits a reset sequence.

## Runtime configuration

Set `SIMULATION_GEARS_LOG_LEVEL` and explicitly call
`logger.setLevelFromEnvironment()` to select a case-insensitive named level or
its numeric value. Invalid values are ignored and preserve the current level.
Explicit constructor settings remain authoritative until that method is called.

```cpp
#include <utils/logging/CLogger.h>

simulation_gears::logging::CLogger logger("navigation");
logger.setLevelFromEnvironment();
logger.error("state propagation failed");
```
