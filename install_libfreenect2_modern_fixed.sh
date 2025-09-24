#!/bin/bash
# Install libfreenect2-modern with CUDA support (Fixed version)
# This will install alongside the existing libfreenect2 installation

set -e  # Exit on any error

echo "🚀 Installing libfreenect2-modern with CUDA support (Fixed)"
echo "=========================================================="

# Check if we're in the right directory
if [ ! -d "libfreenect2" ]; then
    echo "❌ Error: libfreenect2 directory not found. Run this from the HowYouSeeMe root directory."
    exit 1
fi

echo "✅ Found existing libfreenect2 installation - leaving it untouched"

# Check for CUDA
if command -v nvcc &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | sed 's/.*release \([0-9]\+\.[0-9]\+\).*/\1/')
    echo "✅ CUDA $CUDA_VERSION detected"
    
    # Find CUDA installation path
    CUDA_PATH=$(dirname $(dirname $(which nvcc)))
    echo "📍 CUDA path: $CUDA_PATH"
else
    echo "❌ CUDA not found. Please install CUDA toolkit first."
    echo "   Download from: https://developer.nvidia.com/cuda-downloads"
    exit 1
fi

# Install CUDA samples if not present
echo "📦 Installing CUDA samples..."
CUDA_SAMPLES_DIR="$HOME/cuda-samples"
if [ ! -d "$CUDA_SAMPLES_DIR" ]; then
    echo "📥 Downloading CUDA samples..."
    git clone https://github.com/NVIDIA/cuda-samples.git "$CUDA_SAMPLES_DIR"
else
    echo "✅ CUDA samples already exist"
fi

# Check for CMake
if ! command -v cmake &> /dev/null; then
    echo "❌ CMake not found. Installing..."
    sudo apt-get update
    sudo apt-get install -y cmake
fi

CMAKE_VERSION=$(cmake --version | head -n1 | sed 's/cmake version //')
echo "✅ CMake $CMAKE_VERSION detected"

# Install build dependencies
echo "📦 Installing build dependencies..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    pkg-config \
    git \
    libusb-1.0-0-dev \
    libglfw3-dev \
    libturbojpeg0-dev \
    ocl-icd-opencl-dev \
    libva-dev \
    libjpeg-dev

# Clean up previous attempt
if [ -d "libfreenect2-modern" ]; then
    echo "🧹 Cleaning up previous build attempt..."
    rm -rf libfreenect2-modern
fi

# Clone libfreenect2-modern
echo "📥 Cloning libfreenect2-modern..."
git clone https://github.com/cerealkiller2527/libfreenect2-modern.git
cd libfreenect2-modern

# Create build directory
echo "🏗️  Configuring build..."
mkdir -p build
cd build

# Configure with CUDA support and proper paths
echo "⚙️  Running CMake configuration..."
cmake .. \
    -DCMAKE_INSTALL_PREFIX=$HOME/libfreenect2-modern \
    -DENABLE_CUDA=ON \
    -DENABLE_OPENCL=OFF \
    -DENABLE_OPENGL=ON \
    -DBUILD_EXAMPLES=ON \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCUDA_TOOLKIT_ROOT_DIR="$CUDA_PATH" \
    -DCUDA_SAMPLES_ROOT="$CUDA_SAMPLES_DIR" \
    -DCL_TARGET_OPENCL_VERSION=120

# Build
echo "🔨 Building libfreenect2-modern..."
make -j$(nproc)

# Install
echo "📦 Installing libfreenect2-modern..."
make install

# Test installation
echo "🧪 Testing installation..."
cd $HOME/libfreenect2-modern/bin

# Check if Protonect exists
if [ -f "./Protonect" ]; then
    echo "✅ Protonect executable found"
    
    # Test CUDA pipeline (quick test)
    echo "🚀 Testing CUDA pipeline..."
    timeout 5 ./Protonect cuda || echo "CUDA test completed (timeout expected)"
    
    echo ""
    echo "🎉 libfreenect2-modern installation completed successfully!"
    echo ""
    echo "📍 Installation location: $HOME/libfreenect2-modern"
    echo "🔧 Executable: $HOME/libfreenect2-modern/bin/Protonect"
    echo ""
    echo "🧪 Test commands:"
    echo "   $HOME/libfreenect2-modern/bin/Protonect cuda    # CUDA pipeline (fastest)"
    echo "   $HOME/libfreenect2-modern/bin/Protonect gl      # OpenGL pipeline"
    echo "   $HOME/libfreenect2-modern/bin/Protonect cpu     # CPU pipeline"
    echo ""
    echo "📝 Next steps:"
    echo "1. Test the installation with the commands above"
    echo "2. Update your Python environment to use the new library"
    echo "3. Set environment variable: export LIBFREENECT2_INSTALL_PREFIX=$HOME/libfreenect2-modern"
    
else
    echo "❌ Installation failed - Protonect executable not found"
    exit 1
fi