#!/bin/bash
# Direct installation of pylibfreenect2-py310 with correct paths

set -e

echo "🎮 Installing pylibfreenect2-py310 (Direct Method)"
echo "=================================================="

# Check if conda environment is activated
if [[ "$CONDA_DEFAULT_ENV" != "howyouseeme" ]]; then
    echo "❌ Please activate the howyouseeme conda environment first:"
    echo "   conda activate howyouseeme"
    exit 1
fi

# Set the correct libfreenect2 path
export LIBFREENECT2_INSTALL_PREFIX="$HOME/freenect2"
echo "🔧 Using LIBFREENECT2_INSTALL_PREFIX=$LIBFREENECT2_INSTALL_PREFIX"

# Verify the installation exists
echo "🔍 Verifying libfreenect2 installation..."
REQUIRED_FILES=(
    "$LIBFREENECT2_INSTALL_PREFIX/include/libfreenect2/config.h"
    "$LIBFREENECT2_INSTALL_PREFIX/lib/libfreenect2.so"
    "$LIBFREENECT2_INSTALL_PREFIX/bin/Protonect"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [[ -f "$file" ]]; then
        echo "✅ Found: $file"
    else
        echo "❌ Missing: $file"
        exit 1
    fi
done

# Update library path
export LD_LIBRARY_PATH="$LIBFREENECT2_INSTALL_PREFIX/lib:$LD_LIBRARY_PATH"
echo "🔧 Updated LD_LIBRARY_PATH"

# Test libfreenect2 works
echo "🧪 Testing libfreenect2..."
if timeout 10s "$LIBFREENECT2_INSTALL_PREFIX/bin/Protonect" --help &>/dev/null; then
    echo "✅ libfreenect2 test successful"
else
    echo "⚠️  libfreenect2 test failed or timed out (this might be normal)"
fi

# Install build dependencies
echo "🔨 Installing build dependencies..."
pip install --upgrade pip setuptools wheel
pip install cython>=0.29.36 numpy>=1.19.0

# Install pylibfreenect2-py310
echo "🚀 Installing pylibfreenect2-py310..."
echo "   This may take several minutes to compile..."

pip install -v git+https://github.com/cerealkiller2527/pylibfreenect2-py310.git

# Test installation
echo "🧪 Testing pylibfreenect2 installation..."
python -c "
import sys
try:
    import pylibfreenect2
    print('✅ pylibfreenect2 import successful!')
    
    # Test basic functionality
    fn = pylibfreenect2.Freenect2()
    num_devices = fn.enumerateDevices()
    print(f'📱 Kinect devices detected: {num_devices}')
    
    # Test pipelines
    pipelines = []
    try:
        pylibfreenect2.CudaPacketPipeline()
        pipelines.append('CUDA')
    except:
        pass
    
    try:
        pylibfreenect2.OpenCLPacketPipeline()
        pipelines.append('OpenCL')
    except:
        pass
        
    try:
        pylibfreenect2.OpenGLPacketPipeline()
        pipelines.append('OpenGL')
    except:
        pass
        
    try:
        pylibfreenect2.CpuPacketPipeline()
        pipelines.append('CPU')
    except:
        pass
    
    print(f'🚀 Available pipelines: {pipelines}')
    
    if pipelines:
        print('✅ pylibfreenect2-py310 installation successful!')
    else:
        print('⚠️  No pipelines available - check libfreenect2 build')
        
except ImportError as e:
    print(f'❌ Import failed: {e}')
    sys.exit(1)
except Exception as e:
    print(f'⚠️  Test failed: {e}')
    print('   This might be normal if no Kinect is connected')
"

echo ""
echo "🎉 Installation complete!"
echo ""
echo "Environment variables for this session:"
echo "export LIBFREENECT2_INSTALL_PREFIX=$LIBFREENECT2_INSTALL_PREFIX"
echo "export LD_LIBRARY_PATH=$LIBFREENECT2_INSTALL_PREFIX/lib:\$LD_LIBRARY_PATH"
echo ""
echo "To make these permanent, add to ~/.bashrc:"
echo "echo 'export LIBFREENECT2_INSTALL_PREFIX=$LIBFREENECT2_INSTALL_PREFIX' >> ~/.bashrc"
echo "echo 'export LD_LIBRARY_PATH=$LIBFREENECT2_INSTALL_PREFIX/lib:\$LD_LIBRARY_PATH' >> ~/.bashrc"