#!/bin/bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
cd "$SCRIPT_DIR"

CLEAN=false
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
JOBS="${JOBS:-$(nproc)}"
PREFIX="${BOTCREW_PREFIX:-/opt/botcrew}"

for arg in "$@"; do
    case $arg in
        --clean) CLEAN=true ;;
        --release) BUILD_TYPE="Release" ;;
        --debug) BUILD_TYPE="Debug" ;;
    esac
done

if [ ! -d "${PREFIX}/lib/cmake/message_manager" ]; then
    echo "[BUILD] FATAL: message_manager not found at ${PREFIX}." >&2
    echo "[BUILD] Run ./setup/install.sh first." >&2
    exit 1
fi

if $CLEAN && [ -d build ]; then
    rm -rf build
fi

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DCMAKE_PREFIX_PATH="$PREFIX" \
      ..
make -j"$JOBS"

echo "Build complete: $BUILD_TYPE"
