
# ===========================================
# Simple run script without taking portability into account.
#rm -rf build
#cmake -Bbuild -DCMAKE_PREFIX_PATH=/opt/unitree_robotics
#cd build 
#make 
#cd bin
#./AbdullahElnahrawy_g1_motions enp7s0
# ===========================================


#!/usr/bin/env bash

# ===========================================
# Build & Run Script for G1 Motions (Unitree)
# ===========================================
set -e  # Exit immediately on error

DEFAULT_IFACE="enp7s0"
DEFAULT_SDK_PATH="/opt/unitree_robotics"

# --- Get Network Interface ---
if [[ $# -ge 1 ]]; then
    INTERFACE_NAME="$1"
    echo "Using network interface: $INTERFACE_NAME"
else
    echo "Please enter your network interface name [default: $DEFAULT_IFACE]"
    echo "Tip: run 'ifconfig' to list available interfaces."
    read -r INTERFACE_NAME
    INTERFACE_NAME=${INTERFACE_NAME:-$DEFAULT_IFACE}
fi

# --- Clean previous build if exists ---
if [[ -d "build" ]]; then
    echo "Removing previous build directory..."
    rm -rf build
fi

# --- First attempt with default & standard paths ---
echo "Attempting build with default Unitree SDK path: $DEFAULT_SDK_PATH"
if ! cmake -B build -DCMAKE_PREFIX_PATH="$DEFAULT_SDK_PATH"; then
    echo "First attempt failed: Unitree SDK not found in standard paths or $DEFAULT_SDK_PATH"
    read -p "Please enter the correct Unitree SDK path: " USER_SDK_PATH
    if [[ ! -d "$USER_SDK_PATH" ]]; then
        echo "Error: Provided path '$USER_SDK_PATH' does not exist."
        exit 1
    fi
    echo "Retrying build with user-provided SDK path: $USER_SDK_PATH"
    rm -rf build
    cmake -B build -DCMAKE_PREFIX_PATH="$USER_SDK_PATH"
fi

# --- Build project ---
echo "Building project..."
cmake --build build -- -j"$(nproc)"

# --- Run executable ---
EXECUTABLE="./AbdullahElnahrawy_g1_motions"
cd build/bin
if [[ ! -x "$EXECUTABLE" ]]; then
    echo "Error: Executable '$EXECUTABLE' not found or not executable."
    exit 1
fi

echo "Running program with interface: $INTERFACE_NAME"
"$EXECUTABLE" "$INTERFACE_NAME"


