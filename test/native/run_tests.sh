#!/usr/bin/env bash
# Builds and runs every test/native/test_*.{c,cpp}:
#  - test_previpass_*.c against the real, unmodified previpass.c /
#    previpass_util.c from the repo root.
#  - test_parsers.cpp / test_gps_mask.cpp / test_misc.cpp against the pure
#    logic extracted into ../../src/logic/ (parsers.cpp, gps_mask.cpp,
#    misc.cpp) and wired into popup-buoy.ino. src/ is the Arduino-standard
#    subfolder for extra sketch source files that need real subdirectory
#    structure (the sketch's own top-level folder is not compiled
#    recursively, but src/ is).
# No PlatformIO, no Arduino core -- just gcc/g++ and libm. Each test file
# is its own executable (see framework.h for why).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOGIC_DIR="${REPO_ROOT}/src/logic"
BUILD_DIR="${SCRIPT_DIR}/.build"

CC="${CC:-gcc}"
CXX="${CXX:-g++}"
SANITIZE="-fsanitize=address,undefined -static-libasan"

CFLAGS="-std=c11 -Wall -Wextra -I${REPO_ROOT} -I${SCRIPT_DIR} ${SANITIZE} -g -O1"
CXXFLAGS="-std=c++17 -Wall -Wextra -I${LOGIC_DIR} -I${SCRIPT_DIR} ${SANITIZE} -g -O1"

# test_previpass_known_issues.c intentionally exercises a real out-of-bounds
# read in the vendored previpass_util.c (see that file's header comment).
# AddressSanitizer treats it as fatal (process abort); UBSan alone reports
# it to stderr and continues, which is what we want here: the violation
# still gets surfaced in the log, but doesn't take the whole suite down.
KNOWN_ISSUES_CFLAGS="-std=c11 -Wall -Wextra -I${REPO_ROOT} -I${SCRIPT_DIR} -fsanitize=undefined -g -O1"

mkdir -p "${BUILD_DIR}"

overall_status=0

build_and_run() {
  local name="$1"
  shift
  echo "==> building ${name}"
  "$@"
  echo "==> running ${name}"
  if ! "${BUILD_DIR}/${name}"; then
    overall_status=1
  fi
  echo
}

for test_src in "${SCRIPT_DIR}"/test_previpass_*.c; do
  name="$(basename "${test_src}" .c)"
  bin="${BUILD_DIR}/${name}"

  if [ "${name}" = "test_previpass_known_issues" ]; then
    flags="${KNOWN_ISSUES_CFLAGS}"
  else
    flags="${CFLAGS}"
  fi

  build_and_run "${name}" \
    "${CC}" ${flags} -o "${bin}" "${test_src}" \
    "${REPO_ROOT}/previpass.c" "${REPO_ROOT}/previpass_util.c" -lm
done

for test_src in "${SCRIPT_DIR}"/test_parsers.cpp "${SCRIPT_DIR}"/test_gps_mask.cpp "${SCRIPT_DIR}"/test_misc.cpp; do
  name="$(basename "${test_src}" .cpp)"
  bin="${BUILD_DIR}/${name}"

  build_and_run "${name}" \
    "${CXX}" ${CXXFLAGS} -o "${bin}" "${test_src}" \
    "${LOGIC_DIR}/parsers.cpp" "${LOGIC_DIR}/gps_mask.cpp" "${LOGIC_DIR}/misc.cpp"
done

if [ "${overall_status}" -eq 0 ]; then
  echo "ALL NATIVE TESTS PASSED"
else
  echo "NATIVE TESTS FAILED" >&2
fi

exit "${overall_status}"
