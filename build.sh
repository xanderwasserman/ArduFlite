#!/bin/bash

# ArduFlite Build Script
#
# Usage:
#   ./build.sh [fire|lolin]          # Full build with web server
#   ./build.sh [fire|lolin] lite     # Minimal build without web server (~500KB smaller)
#
# Examples:
#   ./build.sh lolin           # Build for Lolin C3 Mini with web server
#   ./build.sh lolin lite      # Build for Lolin C3 Mini without web server
#   ./build.sh fire            # Build for FireBeetle2 with web server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOARD="$1"
VARIANT="$2"
BUILD_PATH="./build"
OPT_FLAGS="-Os"  # Optimize for size

# Parse board (using no_ota partition: 2MB app + 1.9MB LittleFS for flight logs)
if [[ "$BOARD" == "fire" ]]; then
    FQBN="esp32:esp32:dfrobot_firebeetle2_esp32e:PartitionScheme=no_ota"
elif [[ "$BOARD" == "lolin" ]]; then
    FQBN="esp32:esp32:lolin_c3_mini:PartitionScheme=no_ota"
else
    echo "Usage: $0 [fire|lolin] [lite]"
    echo ""
    echo "Boards:"
    echo "  fire   - DFRobot FireBeetle2 ESP32-E"
    echo "  lolin  - Lolin C3 Mini"
    echo ""
    echo "Variants:"
    echo "  (none) - Full build with web server (default)"
    echo "  lite   - Minimal build without web server (~500KB smaller)"
    exit 1
fi

# Parse variant
if [[ "$VARIANT" == "lite" ]]; then
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║  Building LITE variant (web server disabled)                      ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    EXTRA_FLAGS="-DENABLE_WEB_SERVER=0"
else
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║  Building FULL variant (web server enabled)                       ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    
    # Compress web assets before building
    WEB_UI_DIR="$SCRIPT_DIR/tools/web_ui"
    if [[ -d "$WEB_UI_DIR/src" ]]; then
        echo "Compressing web assets..."
        pushd "$WEB_UI_DIR" > /dev/null
        python3 compress.py > "$SCRIPT_DIR/src/web/WebUI.h"
        popd > /dev/null
        echo "Web assets compressed successfully."
    else
        echo "Warning: tools/web_ui/src not found, skipping compression"
    fi
fi

echo ""
echo "Board: $FQBN"
[[ -n "$EXTRA_FLAGS" ]] && echo "Extra flags: $EXTRA_FLAGS"
echo ""

BUILD_PROPS=(
    --build-property "build.optimization_flags=$OPT_FLAGS"
)
[[ -n "$EXTRA_FLAGS" ]] && BUILD_PROPS+=(--build-property "build.extra_flags=$EXTRA_FLAGS")

arduino-cli compile \
    -b "$FQBN" \
    --warnings default \
    --build-path "$BUILD_PATH" \
    "${BUILD_PROPS[@]}" \
    --verbose
