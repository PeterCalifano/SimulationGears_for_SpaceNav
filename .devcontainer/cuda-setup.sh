#!/usr/bin/env bash
# Install CUDA for standalone image builds. Dev Containers use the CUDA
# feature configured in devcontainer.json instead.
set -euo pipefail

install_cuda="${INSTALL_CUDA:-off}"
cuda_version="${CUDA_VERSION:-12.9}"

if [[ "$install_cuda" != "on" ]]; then
  echo "cuda-setup.sh: INSTALL_CUDA is not 'on', skipping CUDA toolkit install."
  exit 0
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "cuda-setup.sh: apt-get not found, cannot install CUDA toolkit." >&2
  exit 1
fi
if [[ ! -r /etc/os-release ]]; then
  echo "cuda-setup.sh: /etc/os-release not found, cannot detect base OS." >&2
  exit 1
fi

# shellcheck source=/etc/os-release
source /etc/os-release
case "${ID:-}-${VERSION_ID:-}" in
  ubuntu-24.04) repository_tag="ubuntu2404" ;;
  ubuntu-22.04) repository_tag="ubuntu2204" ;;
  ubuntu-20.04) repository_tag="ubuntu2004" ;;
  debian-12) repository_tag="debian12" ;;
  *)
    echo "cuda-setup.sh: no CUDA apt repository mapping for '${ID:-?} ${VERSION_ID:-?}'." >&2
    exit 1
    ;;
esac

architecture="$(dpkg --print-architecture)"
case "$architecture" in
  amd64) repository_architecture="x86_64" ;;
  arm64) repository_architecture="sbsa" ;;
  *)
    echo "cuda-setup.sh: unsupported architecture '${architecture}'." >&2
    exit 1
    ;;
esac

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl gnupg

keyring_package="/tmp/cuda-keyring.deb"
curl -fsSL \
  "https://developer.download.nvidia.com/compute/cuda/repos/${repository_tag}/${repository_architecture}/cuda-keyring_1.1-1_all.deb" \
  -o "$keyring_package"
dpkg -i "$keyring_package"
rm -f "$keyring_package"

cuda_package="cuda-toolkit-${cuda_version//./-}"
apt-get update
apt-get install -y --no-install-recommends "$cuda_package"

apt-get clean
rm -rf /var/lib/apt/lists/*
