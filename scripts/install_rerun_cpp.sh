#!/bin/bash
# Install Rerun C++ SDK

set -e

echo "Installing Rerun C++ SDK..."

# Create temp directory
TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"

# Download latest Rerun C++ SDK
echo "Downloading Rerun C++ SDK..."
wget https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip

# Extract
echo "Extracting..."
unzip -q rerun_cpp_sdk.zip

# Detect architecture
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    LIB_FILE="librerun_c__linux_x64.a"
elif [ "$ARCH" = "aarch64" ]; then
    LIB_FILE="librerun_c__linux_arm64.a"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

echo "Detected architecture: $ARCH"
echo "Using library: $LIB_FILE"

# Install to /usr/local
echo "Installing to /usr/local..."
sudo mkdir -p /usr/local/include/rerun
sudo cp -r rerun_cpp_sdk/src/rerun/* /usr/local/include/rerun/
sudo cp rerun_cpp_sdk/src/rerun.hpp /usr/local/include/
sudo cp "rerun_cpp_sdk/lib/$LIB_FILE" /usr/local/lib/librerun_c.a

# Update library cache
sudo ldconfig

# Cleanup
cd -
rm -rf "$TEMP_DIR"

echo "✓ Rerun C++ SDK installed successfully"
echo ""
echo "Headers: /usr/local/include/rerun/"
echo "Library: /usr/local/lib/librerun_c.a"
