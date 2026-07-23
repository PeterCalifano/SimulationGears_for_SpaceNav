#!/usr/bin/env bash
# Build and run the SimulationGears development image without requiring the
# Dev Containers CLI. The repository is mounted read-write at /workspace.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_NAME="$(basename "$ROOT_DIR")"
IMAGE_TAG="$(echo "$REPOSITORY_NAME" | tr '[:upper:]' '[:lower:]')-dev:latest"
ENGINE=""
FORCE_BUILD="no"
USE_GPU="yes"
CUDA_VERSION="12.9"

usage() {
  cat <<EOF
Usage: ./run_in_container.sh [options] [--] [command [args...]]

Runs a command in the repository development image (default: bash).

Options:
  --build              Force a rebuild of the image.
  --no-gpu             Do not request GPU access.
  --image <name>       Override the image tag (default: ${IMAGE_TAG}).
  --engine <engine>    Select docker or podman (default: autodetect).
  --cuda-version <v>   CUDA toolkit build argument (default: ${CUDA_VERSION}).
  -h, --help           Show this help.

Docker uses --gpus all. Podman uses NVIDIA CDI and disables SELinux relabeling
for the GPU device; generate the CDI specification with nvidia-ctk if needed.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) FORCE_BUILD="yes" ;;
    --no-gpu) USE_GPU="no" ;;
    --image)
      shift
      IMAGE_TAG="${1:-}"
      [[ -n "$IMAGE_TAG" ]] || { echo "--image requires a value."; exit 1; }
      ;;
    --engine)
      shift
      ENGINE="${1:-}"
      case "$ENGINE" in
        docker | podman) ;;
        *) echo "--engine must be 'docker' or 'podman'."; exit 1 ;;
      esac
      ;;
    --cuda-version)
      shift
      CUDA_VERSION="${1:-}"
      [[ -n "$CUDA_VERSION" ]] || { echo "--cuda-version requires a value."; exit 1; }
      ;;
    -h | --help) usage; exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
  shift
done

if [[ -z "$ENGINE" ]]; then
  if command -v docker >/dev/null 2>&1; then
    ENGINE="docker"
  elif command -v podman >/dev/null 2>&1; then
    ENGINE="podman"
  else
    echo "Neither docker nor podman was found in PATH."
    exit 1
  fi
fi

need_build="$FORCE_BUILD"
if [[ "$need_build" != "yes" ]] && ! "$ENGINE" image inspect "$IMAGE_TAG" >/dev/null 2>&1; then
  need_build="yes"
fi
if [[ "$need_build" == "yes" ]]; then
  echo "Building ${IMAGE_TAG} with ${ENGINE} (CUDA ${CUDA_VERSION})..."
  "$ENGINE" build \
    --build-arg INSTALL_CUDA=on \
    --build-arg CUDA_VERSION="$CUDA_VERSION" \
    -t "$IMAGE_TAG" \
    "$ROOT_DIR/.devcontainer"
fi

gpu_arguments=()
if [[ "$USE_GPU" == "yes" ]]; then
  if [[ "$ENGINE" == "docker" ]]; then
    gpu_arguments=(--gpus all)
  else
    gpu_arguments=(--device nvidia.com/gpu=all --security-opt=label=disable)
  fi
fi

tty_arguments=()
if [[ -t 0 && -t 1 ]]; then
  tty_arguments=(-it)
fi
if [[ $# -eq 0 ]]; then
  set -- bash
fi

exec "$ENGINE" run --rm "${tty_arguments[@]}" \
  "${gpu_arguments[@]}" \
  -v "$ROOT_DIR":/workspace \
  -w /workspace \
  -e DISPLAY="${DISPLAY:-}" \
  "$IMAGE_TAG" "$@"
