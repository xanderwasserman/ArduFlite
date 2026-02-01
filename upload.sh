#!/bin/bash

# Usage: ./upload.sh [firebeetle2|lolin_c3]

set -e

BOARD="$1"

if [[ -z "$BOARD" ]]; then
    echo "Usage: $0 [firebeetle2|lolin_c3]"
    exit 1
fi

if [[ "$BOARD" == "fire" ]]; then
    FQBN="esp32:esp32:dfrobot_firebeetle2_esp32e"
elif [[ "$BOARD" == "lolin" ]]; then
    FQBN="esp32:esp32:lolin_c3_mini"
else
    echo "Invalid board: $BOARD"
    echo "Valid options: fire, lolin"
    exit 1
fi

# Detect OS and set port detection logic
OS="$(uname)"
PORT=""

if [[ "$OS" == "Darwin" ]]; then
    # macOS
    PORT=$(ls /dev/tty.* | grep -E 'usbserial|usbmodem|SLAB_USBtoUART' | head -n 1)
elif [[ "$OS" == "Linux" ]]; then
    PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)
elif [[ "$OS" =~ "MINGW" || "$OS" =~ "MSYS" || "$OS" =~ "CYGWIN" ]]; then
    # Git Bash on Windows
    PORT=$(ls /dev/ttyS* 2>/dev/null | head -n 1)
else
    echo "Unsupported OS: $OS"
    exit 1
fi

if [[ -z "$PORT" ]]; then
    echo "No serial port detected. Please plug in your board or specify manually."
    exit 1
fi

echo "Detected port: $PORT"

BUILD_PATH="./build"

arduino-cli upload -p "$PORT" -b "$FQBN" --input-dir "$BUILD_PATH"
