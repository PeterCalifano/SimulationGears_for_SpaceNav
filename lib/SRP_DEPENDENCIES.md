# Private SRP file-boundary dependencies

These sources are vendored, not fetched at configure time or exposed in installed public headers.
No parser types enter the numerical kernels.

| Directory | Source and revision | License | Preparation |
| --- | --- | --- | --- |
| rapidjson | https://github.com/Tencent/rapidjson at `24b5e7a8b27f42fa16b96fc70aade9106cf7102f` | MIT (see LICENSE) | Unmodified include/rapidjson directory |
| rapidyaml | https://github.com/biojppm/rapidyaml at `c42152f2b7b8c7a450a670d61e890d9995088109` | MIT (see LICENSE and embedded licenses) | `python3 tools/amalgamate.py ryml_all.hpp`, default c4core, STL and fastfloat inclusion |
| picosha2 | https://github.com/okdshin/PicoSHA2 at `27fcf6979298949e8a462e16d09a0351c18fcaf2` | MIT (embedded) | Unmodified picosha2.h, used only to check descriptor/mesh identities |

RapidYAML's self-contained header includes implementation. `SrpPanelInput.cpp` defines
`RYML_SINGLE_HDR_DEFINE_NOW` exactly once and enables exception-based errors; it does not alter
process-global callbacks. The parsers and SHA implementation require no additional installed
library. Their upstream naming and formatting are intentionally preserved.
