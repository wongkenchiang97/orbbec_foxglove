#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-build-ninja-linux}"
CMAKE_EXE="${CMAKE_EXE:-cmake}"
NINJA_EXE="${NINJA_EXE:-ninja}"
ORBBEC_SDK_ROOT="${ORBBEC_SDK_ROOT:-/opt/orbbec-sdk}"
FOXGLOVE_SDK_ROOT="${FOXGLOVE_SDK_ROOT:-${SCRIPT_DIR}/../foxglove-sdk}"
VCPKG_TOOLCHAIN_DEFAULT="${SCRIPT_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake"
VCPKG_TOOLCHAIN="${VCPKG_TOOLCHAIN:-${VCPKG_TOOLCHAIN_DEFAULT}}"
VCPKG_TARGET_TRIPLET="${VCPKG_TARGET_TRIPLET:-x64-linux}"
CONFIGURE_ONLY=0

print_usage() {
  cat <<'EOF'
Usage: ./build_ninja_linux.sh [--configure-only]

Environment overrides:
  ORBBEC_SDK_ROOT      Default: /opt/orbbec-sdk
  FOXGLOVE_SDK_ROOT    Default: ../foxglove-sdk (relative to this script)
  BUILD_DIR            Default: build-ninja-linux
  CMAKE_EXE            Default: cmake
  NINJA_EXE            Default: ninja
  VCPKG_TOOLCHAIN      Optional path to vcpkg toolchain
  VCPKG_TARGET_TRIPLET Default: x64-linux

Examples:
  ORBBEC_SDK_ROOT=/opt/OrbbecSDK FOXGLOVE_SDK_ROOT=$HOME/dev/foxglove-sdk ./build_ninja_linux.sh
  ./build_ninja_linux.sh --configure-only
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --configure-only)
      CONFIGURE_ONLY=1
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      print_usage
      exit 2
      ;;
  esac
done

if [[ ! -f "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp" ]]; then
  echo "ERROR: ORBBEC_SDK_ROOT is invalid: ${ORBBEC_SDK_ROOT}" >&2
  echo "Expected: ${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp" >&2
  exit 1
fi

if [[ ! -f "${FOXGLOVE_SDK_ROOT}/include/foxglove/server.hpp" ]]; then
  echo "ERROR: FOXGLOVE_SDK_ROOT is invalid: ${FOXGLOVE_SDK_ROOT}" >&2
  echo "Expected: ${FOXGLOVE_SDK_ROOT}/include/foxglove/server.hpp" >&2
  exit 1
fi

CONFIGURE_ARGS=(
  -S "${SCRIPT_DIR}"
  -B "${SCRIPT_DIR}/${BUILD_DIR}"
  -G Ninja
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_MAKE_PROGRAM="${NINJA_EXE}"
  -DORBBEC_SDK_ROOT="${ORBBEC_SDK_ROOT}"
  -DFOXGLOVE_SDK_ROOT="${FOXGLOVE_SDK_ROOT}"
)

if [[ -f "${VCPKG_TOOLCHAIN}" ]]; then
  CONFIGURE_ARGS+=(
    -DCMAKE_TOOLCHAIN_FILE="${VCPKG_TOOLCHAIN}"
    -DVCPKG_TARGET_TRIPLET="${VCPKG_TARGET_TRIPLET}"
  )
fi

"${CMAKE_EXE}" "${CONFIGURE_ARGS[@]}"

if [[ "${CONFIGURE_ONLY}" -eq 0 ]]; then
  "${CMAKE_EXE}" --build "${SCRIPT_DIR}/${BUILD_DIR}"
fi
