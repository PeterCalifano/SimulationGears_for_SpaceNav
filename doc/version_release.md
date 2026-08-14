# Version and source release contract

## Version ownership

Version resolution follows Git tags, then a source `VERSION` file, then the
hardcoded CMake fallback. ROS 2 manifests always receive the strict core
`X.Y.Z`; prerelease and build metadata are retained only in the full version.
CPack binary and source package filenames use that full version. The source
archive appends `-source`, so generating both package kinds cannot overwrite
one artifact with the other.

Every generated `VERSION` file contains exactly these five fields and one
terminating newline, with no blank sixth line:

```text
Project version: X.Y.Z
Project version core: X.Y.Z
Project version prerelease: <none>
Project version metadata: <none>
Full version: X.Y.Z
```

The canonical source distribution is CPack's generated source TGZ. GitHub's
automatic tag archives are non-canonical because they do not contain the
generated `VERSION` contract. Upload the TGZ to the GitHub release manually.

## Exact release order

For a release `X.Y.Z`, start from a clean intended release commit and use a
temporary local tag only to prepare metadata:

```bash
git tag --no-sign vX.Y.Z
./generate_version.sh --sync-ros2
git tag -d vX.Y.Z
git diff --check
git diff -- ros2/*/package.xml
```

Review and commit only the synchronized ROS manifest metadata. Then create the
final annotated tag on that metadata commit, regenerate the source VERSION, and
validate the tag before packaging:

```bash
git add ros2/*/package.xml
git commit -m "Synchronize ROS metadata for vX.Y.Z"
git diff --exit-code -- ros2/*/package.xml
git tag -a vX.Y.Z -m "SimulationGears vX.Y.Z"
test "$(git describe --tags --exact-match)" = "vX.Y.Z"
./generate_version.sh --sync-ros2
git diff --exit-code -- ros2/*/package.xml
cmake -S . -B build_release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=OFF -DENABLE_SUBMODULES=OFF \
  -DSimulationGears_for_SpaceNav_ENABLE_CUDA=OFF \
  -DSimulationGears_for_SpaceNav_ENABLE_OPTIX=OFF
cmake --build build_release --target package_source
```

The generated `VERSION` is intentionally ignored in normal development but
must exist before `package_source` on an exact release. Inspect and test the TGZ
before manually attaching it to the GitHub release. Do not move the final tag
after publication.

## Canonical archive contents

The source TGZ includes project sources, tests, documentation, release tools,
ROS 2 manifests, and `VERSION`. It excludes Git/VCS data, all build and install
trees, ROS generated trees, Python caches, generated wrapper/codegen products,
and the complete `lib/MathCore_for_ComputerVision` submodule source. Consumers
obtain nested dependencies independently; the archive must configure outside a
Git worktree with `ENABLE_SUBMODULES=OFF`.

## Python wrapper version boundary

The semantic CMake and release version remains the five-field SemVer contract
above. When the optional Python wrapper is configured, the shared donor wrapper
infrastructure projects those structured fields into a PEP 440-compatible
package version without renaming the existing distribution, import package, or
extension module. This repository adds no independent PEP-version policy or
PEP-specific regression suite.
