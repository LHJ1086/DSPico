#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# DSPico — one-time dependency setup.
#
# Fetches the only vendored dependency (Pico-PIO-USB). The Pico SDK is located
# via the PICO_SDK_PATH environment variable; if you don't have it, either
# install it and export PICO_SDK_PATH, or configure CMake with
# -DPICO_SDK_FETCH_FROM_GIT=ON to have CMake download it.
# ---------------------------------------------------------------------------
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIO_USB_DIR="${REPO_ROOT}/lib/Pico-PIO-USB"
PIO_USB_URL="https://github.com/sekigon-gonnoc/Pico-PIO-USB.git"

mkdir -p "${REPO_ROOT}/lib"

if [ -d "${PIO_USB_DIR}/.git" ]; then
    echo "Pico-PIO-USB already present at ${PIO_USB_DIR}"
else
    echo "Cloning Pico-PIO-USB into ${PIO_USB_DIR}"
    git clone --depth 1 "${PIO_USB_URL}" "${PIO_USB_DIR}"
fi

if [ -z "${PICO_SDK_PATH:-}" ]; then
    cat <<'EOF'

NOTE: PICO_SDK_PATH is not set.
  Either install the Pico SDK and:  export PICO_SDK_PATH=/path/to/pico-sdk
  Or let CMake fetch it:            cmake -B build -DPICO_SDK_FETCH_FROM_GIT=ON
EOF
fi

echo "Setup complete. Build with:"
echo "  cmake -B build -DPICO_BOARD=pico2 && cmake --build build -j"
