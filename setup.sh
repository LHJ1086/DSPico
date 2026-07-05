#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# DSPico — one-time dependency setup.
#
# Fetches the only vendored dependency (Pico-PIO-USB), pins it to a known-good
# commit, and applies this repo's patches to it (see patches/ — currently the
# isochronous-OUT host support the audio path needs). Re-running is safe.
#
# The Pico SDK is located via the PICO_SDK_PATH environment variable; if you
# don't have it, either install it and export PICO_SDK_PATH, or configure CMake
# with -DPICO_SDK_FETCH_FROM_GIT=ON to have CMake download it.
# ---------------------------------------------------------------------------
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIO_USB_DIR="${REPO_ROOT}/lib/Pico-PIO-USB"
PIO_USB_URL="https://github.com/sekigon-gonnoc/Pico-PIO-USB.git"
# Pinned so the patches below always apply. Bump deliberately, re-testing the
# patches, rather than tracking a moving master.
PIO_USB_COMMIT="dc9193fed510da5f81f4db520e98282f61db9054"

mkdir -p "${REPO_ROOT}/lib"

if [ -d "${PIO_USB_DIR}/.git" ]; then
    echo "Pico-PIO-USB already present at ${PIO_USB_DIR}"
else
    echo "Cloning Pico-PIO-USB into ${PIO_USB_DIR}"
    git clone "${PIO_USB_URL}" "${PIO_USB_DIR}"
fi

# Pin to the known-good commit (fetch it if the local clone doesn't have it).
if ! git -C "${PIO_USB_DIR}" cat-file -e "${PIO_USB_COMMIT}^{commit}" 2>/dev/null; then
    git -C "${PIO_USB_DIR}" fetch origin "${PIO_USB_COMMIT}"
fi
if [ "$(git -C "${PIO_USB_DIR}" rev-parse HEAD)" != "${PIO_USB_COMMIT}" ]; then
    # A previously patched tree can't switch commits with dirty files; patches
    # are re-applied below, so a hard reset is safe here.
    git -C "${PIO_USB_DIR}" checkout --force --detach "${PIO_USB_COMMIT}"
fi

# Apply this repo's patches (idempotent: skip any patch already applied).
for patch in "${REPO_ROOT}"/patches/*.patch; do
    [ -e "${patch}" ] || continue
    if git -C "${PIO_USB_DIR}" apply --reverse --check "${patch}" 2>/dev/null; then
        echo "Patch already applied: $(basename "${patch}")"
    elif git -C "${PIO_USB_DIR}" apply --check "${patch}" 2>/dev/null; then
        git -C "${PIO_USB_DIR}" apply "${patch}"
        echo "Applied patch: $(basename "${patch}")"
    else
        echo "ERROR: patch does not apply cleanly: $(basename "${patch}")" >&2
        echo "       (wrong Pico-PIO-USB commit? expected ${PIO_USB_COMMIT})" >&2
        exit 1
    fi
done

if [ -z "${PICO_SDK_PATH:-}" ]; then
    cat <<'EOF'

NOTE: PICO_SDK_PATH is not set.
  Either install the Pico SDK and:  export PICO_SDK_PATH=/path/to/pico-sdk
  Or let CMake fetch it:            cmake -B build -DPICO_SDK_FETCH_FROM_GIT=ON
EOF
fi

echo "Setup complete. Build with:"
echo "  cmake -B build -DPICO_BOARD=pico2 && cmake --build build -j"
