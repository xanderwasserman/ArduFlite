#!/bin/bash

# Usage: ./build.sh [firebeetle2|lolin_c3]

set -e

BOARD="$1"
BUILD_PATH="./build"
OPT_FLAGS="-O2"

if [[ "$BOARD" == "fire" ]]; then
    FQBN="esp32:esp32:dfrobot_firebeetle2_esp32e"
elif [[ "$BOARD" == "lolin" ]]; then
    FQBN="esp32:esp32:lolin_c3_mini"
else
    echo "Usage: $0 [fire|lolin]"
    exit 1
fi

arduino-cli compile \
    -b "$FQBN" \
    --warnings default \
    --build-path "$BUILD_PATH" \
    --build-property build.optimization_flags="$OPT_FLAGS" \
    --verbose
