#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# DSPico — run all host-side unit tests (no hardware / no Pico SDK needed).
#
#   * peq_test  — parametric-EQ engine (src/dsp_peq.c)
#   * path_test — signal path + cross-core ring (src/signal_path.c)
#   * web_test  — configurator pure logic (web/dspico.js), if Node is present
#
# Usage: tests/run.sh        (from anywhere; paths are resolved from the repo)
# Exits non-zero if any test fails.
# ---------------------------------------------------------------------------
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "${BUILD_DIR}"' EXIT

CC="${CC:-gcc}"
CFLAGS="-O2 -Wall -Wextra -std=c11 -I ${REPO_ROOT}/src"

echo "== Building native tests with ${CC} =="
"${CC}" ${CFLAGS} -o "${BUILD_DIR}/peq_test" \
    "${REPO_ROOT}/tests/peq_test.c" "${REPO_ROOT}/src/dsp_peq.c" -lm
"${CC}" ${CFLAGS} -o "${BUILD_DIR}/path_test" \
    "${REPO_ROOT}/tests/path_test.c" "${REPO_ROOT}/src/signal_path.c" \
    "${REPO_ROOT}/src/dsp_peq.c" -lm

echo
"${BUILD_DIR}/peq_test"
echo
"${BUILD_DIR}/path_test"

echo
if command -v node >/dev/null 2>&1; then
    node "${REPO_ROOT}/tests/web_test.js"
else
    echo "== Skipping web_test.js (Node not found) =="
fi

echo
echo "All test suites passed."
