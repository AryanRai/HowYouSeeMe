# pylibfreenect2-py310

[![License](http://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat)](LICENSE.md)
[![Python Version](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)](https://github.com/cerealkiller2527/pylibfreenect2-py310)

**Python 3.10+ compatible interface for libfreenect2 with GPU acceleration support**

This is an enhanced version of [pylibfreenect2](https://github.com/r9y9/pylibfreenect2) with Python 3.10+ compatibility, improved Windows support, and robust cross-platform builds. Supports CUDA, OpenCL, and OpenGL pipelines for GPU-accelerated Kinect v2 processing.

## 🎯 Part of the GPU-Accelerated Kinect v2 Stack

This is the **Python bindings layer** of the complete Kinect v2 development stack:

| Layer | Project | Description |
|-------|---------|-------------|
| **📦 Foundation** | [libfreenect2-modern](https://github.com/cerealkiller2527/libfreenect2-modern) | Core C++ driver with GPU pipelines |
| **🐍 Python Bindings** | **pylibfreenect2-py310** (this project) | Python 3.10+ interface with automatic pipeline selection |
| **🔧 High-Level API** | [kinect-toolbox-py310](https://github.com/cerealkiller2527/kinect-toolbox-py310) | Easy-to-use Python wrapper with utilities |

## ⚠️ Important Disclaimers

### Build Complexity Warning
**This package requires compilation from source and has complex dependencies.** Installation success depends on:
- ✅ Having libfreenect2 properly built and installed 
- ✅ Correct development environment setup
- ✅ Platform-specific build tools installed
- ✅ Environment variables configured correctly

**Expected Success Rate**: ~40-60% for typical users. **Not recommended for beginners.**

### Platform Support
- ✅ **Windows 10/11**: Tested and working (requires Visual Studio)
- ✅ **Linux (Ubuntu 20.04+)**: Should work (requires build-essential)  
- ⚠️ **macOS**: Untested but should work (requires Xcode)
- ❌ **Python < 3.10**: Not supported

### Performance Expectations
Frame rates vary significantly by GPU pipeline:
- **CUDA**: 25-35 FPS (NVIDIA GPUs only)
- **OpenCL**: 20-30 FPS (Most GPUs)
- **OpenGL**: 15-25 FPS (Graphics cards)  
- **CPU**: 5-15 FPS (Fallback)

## Prerequisites

### 1. libfreenect2 Installation

**You MUST have libfreenect2 built and installed first.** Follow the comprehensive guide:

🔗 **[libfreenect2-modern Installation Guide](https://github.com/cerealkiller2527/libfreenect2-modern)**

**Enhanced by [Madhav Lodha](https://madhavlodha.com)** - Check out my portfolio at [madhavlodha.com](https://madhavlodha.com) for more projects!

This guide covers:
- Building libfreenect2 from source with GPU support
- Installing all required dependencies (CUDA, OpenCL, etc.)
- Verifying your installation works
- Troubleshooting common issues

### 2. Development Environment Setup

#### Windows 10/11
```powershell
# Install Visual Studio Community (free)
# Download from: https://visualstudio.microsoft.com/vs/community/
# ✅ Select "Desktop development with C++" workload
# ✅ Include Windows 10/11 SDK
# ✅ Include CMake tools

# Verify installation
where cl.exe  # Should find compiler after opening "x64 Native Tools Command Prompt"
```

#### Linux (Ubuntu/Debian)
```bash
# Install build tools
sudo apt-get update
sudo apt-get install build-essential python3-dev pkg-config

# Install additional dependencies
sudo apt-get install cmake git wget

# Verify installation  
gcc --version
python3 --version
```

#### Linux (RHEL/CentOS/Fedora)
```bash
# Install build tools
sudo yum groupinstall "Development Tools"  # CentOS/RHEL
# OR
sudo dnf groupinstall "Development Tools"  # Fedora

sudo yum install python3-devel cmake git wget  # CentOS/RHEL
# OR  
sudo dnf install python3-devel cmake git wget  # Fedora
```

#### macOS
```bash
# Install Xcode Command Line Tools
xcode-select --install

# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install additional tools
brew install cmake git wget

# Verify installation
clang --version
python3 --version
```

## Installation

### Step 1: Set Environment Variables

Tell the build system where to find your libfreenect2 installation:

#### Windows (Command Prompt)
```cmd
set LIBFREENECT2_INSTALL_PREFIX=C:\path\to\your\libfreenect2\install
```

#### Windows (PowerShell)  
```powershell
$env:LIBFREENECT2_INSTALL_PREFIX="C:\path\to\your\libfreenect2\install"
```

#### Linux/macOS (Bash/Zsh)
```bash
export LIBFREENECT2_INSTALL_PREFIX=/usr/local  # or your custom path
# Add to ~/.bashrc or ~/.zshrc for persistence:
echo 'export LIBFREENECT2_INSTALL_PREFIX=/usr/local' >> ~/.bashrc
```

### Step 2: Platform-Specific Installation

#### Windows Installation
```cmd
REM ⚠️ CRITICAL: Use "x64 Native Tools Command Prompt for VS 2022"
REM NOT regular Command Prompt or PowerShell!

REM 1. Open Start Menu → Visual Studio 2022 → x64 Native Tools Command Prompt for VS 2022
REM 2. Activate your Python environment (if using conda/venv)
conda activate your-env-name

REM 3. Set environment variable
set LIBFREENECT2_INSTALL_PREFIX=C:\path\to\your\libfreenect2\install

REM 4. Install package
pip install git+https://github.com/cerealkiller2527/pylibfreenect2-py310.git
```

#### Linux Installation
```bash
# 1. Set environment variable
export LIBFREENECT2_INSTALL_PREFIX=/usr/local  # or your path

# 2. Install package
pip install git+https://github.com/cerealkiller2527/pylibfreenect2-py310.git
```

#### macOS Installation
```bash
# 1. Set environment variable  
export LIBFREENECT2_INSTALL_PREFIX=/usr/local  # or your path

# 2. Install package
pip install git+https://github.com/cerealkiller2527/pylibfreenect2-py310.git
```

## Quick Verification

### Test Basic Import
```python
import pylibfreenect2
print("✅ Import successful!")
print("Available pipelines:", [n for n in dir(pylibfreenect2) if 'Pipeline' in n])
```

**Expected Output:**
```
✅ Import successful!
Available pipelines: ['CpuPacketPipeline', 'CudaPacketPipeline', 'OpenGLPacketPipeline', 'PacketPipeline']
```

### Test Device Connection
```python
import pylibfreenect2

# Test device enumeration
fn = pylibfreenect2.Freenect2()
num_devices = fn.enumerateDevices()
print(f"Kinect devices found: {num_devices}")

if num_devices > 0:
    print("✅ Kinect v2 detected and accessible!")
else:
    print("❌ No Kinect v2 devices found")
```

### Test GPU Pipeline Performance
```python
import pylibfreenect2
import time

# Test CUDA pipeline (fastest)
try:
    pipeline = pylibfreenect2.CudaPacketPipeline()
    print("✅ CUDA pipeline available")
except:
    print("❌ CUDA pipeline not available")

# Test OpenCL pipeline (cross-platform)  
try:
    pipeline = pylibfreenect2.OpenCLPacketPipeline()
    print("✅ OpenCL pipeline available")
except:
    print("❌ OpenCL pipeline not available")
```

## Troubleshooting

### Windows Issues

#### "cl.exe failed: None"
- ✅ **Solution**: Use "x64 Native Tools Command Prompt for VS 2022"
- ❌ **Don't use**: Regular Command Prompt, PowerShell, or Git Bash

#### "DLL load failed while importing libfreenect2"
```cmd
REM Check environment variable
echo %LIBFREENECT2_INSTALL_PREFIX%

REM Verify DLL exists
dir "%LIBFREENECT2_INSTALL_PREFIX%\bin\freenect2.dll"

REM If missing, check your libfreenect2 installation
```

#### "error: Microsoft Visual C++ 14.0 is required"
- Install Visual Studio Community with C++ tools
- Ensure you're using the Native Tools Command Prompt

### Linux Issues

#### "OSError: libfreenect2/config.h: is not found"
```bash
# Check environment variable
echo $LIBFREENECT2_INSTALL_PREFIX

# Verify config.h exists
ls $LIBFREENECT2_INSTALL_PREFIX/include/libfreenect2/config.h

# Check library
ls $LIBFREENECT2_INSTALL_PREFIX/lib/libfreenect2*
```

#### "ImportError: libfreenect2.so: cannot open shared object file"
```bash
# Add to library path
export LD_LIBRARY_PATH=$LIBFREENECT2_INSTALL_PREFIX/lib:$LD_LIBRARY_PATH

# Make permanent
echo 'export LD_LIBRARY_PATH=$LIBFREENECT2_INSTALL_PREFIX/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```

### macOS Issues

#### "dyld: Library not loaded: libfreenect2"
```bash
# Add to library path
export DYLD_LIBRARY_PATH=$LIBFREENECT2_INSTALL_PREFIX/lib:$DYLD_LIBRARY_PATH

# Check library exists
ls $LIBFREENECT2_INSTALL_PREFIX/lib/libfreenect2*
```

### Universal Issues

#### "RuntimeError: Cython is required"
- This should auto-install now, but if it fails:
```bash
pip install cython>=0.29.36 numpy>=1.19.0
```

#### "No GPU pipelines available"
- Verify libfreenect2 was built with GPU support:
```bash
# Test libfreenect2 directly first
$LIBFREENECT2_INSTALL_PREFIX/bin/Protonect
# Should show: "CUDA, OpenCL, OpenGL" pipelines available
```

#### Performance is poor (< 10 FPS)
- Check you're using GPU pipeline, not CPU:
```python
# Force CUDA pipeline
pipeline = pylibfreenect2.CudaPacketPipeline()
device = fn.openDefaultDevice(pipeline)
```

## Advanced Usage

### Custom Pipeline Selection
```python
import pylibfreenect2

fn = pylibfreenect2.Freenect2()

# Try pipelines in order of performance
pipelines_to_try = [
    pylibfreenect2.CudaPacketPipeline,      # Fastest (NVIDIA only)
    pylibfreenect2.OpenCLPacketPipeline,    # Fast (most GPUs)
    pylibfreenect2.OpenGLPacketPipeline,    # Moderate (graphics cards)
    pylibfreenect2.CpuPacketPipeline        # Slowest (always works)
]

device = None
for pipeline_class in pipelines_to_try:
    try:
        pipeline = pipeline_class()
        device = fn.openDefaultDevice(pipeline)
        print(f"✅ Using {pipeline_class.__name__}")
        break
    except:
        continue

if device is None:
    print("❌ No working pipeline found!")
```

### Frame Capture Example
```python
import pylibfreenect2
import numpy as np

# Setup
fn = pylibfreenect2.Freenect2()
device = fn.openDefaultDevice()

listener = pylibfreenect2.SyncMultiFrameListener(
    pylibfreenect2.FrameType.Color | pylibfreenect2.FrameType.Depth
)
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()

try:
    frames = {}
    listener.waitForNewFrame(frames)
    
    color = frames[pylibfreenect2.FrameType.Color]
    depth = frames[pylibfreenect2.FrameType.Depth]
    
    print(f"Color frame: {color.width}x{color.height}")
    print(f"Depth frame: {depth.width}x{depth.height}")
    
    # Convert to numpy arrays
    color_array = color.asarray()
    depth_array = depth.asarray()
    
    listener.release(frames)
finally:
    device.stop()
    device.close()
```

## Support and Contribution

### Getting Help
1. **Check this README** first - most issues are covered here
2. **Verify libfreenect2 works** independently before reporting issues  
3. **Include full error messages and system info** when asking for help
4. **Search existing issues** in this repository

### Reporting Issues
When reporting issues, include:
- Operating system and version
- Python version (`python --version`)
- Visual Studio version (Windows)
- Full error message and traceback
- Output of libfreenect2 test: `$LIBFREENECT2_INSTALL_PREFIX/bin/Protonect`

### Contributing
Contributions welcome! This project specifically focuses on:
- Python 3.10+ compatibility
- Windows build improvements  
- Cross-platform reliability
- Better error messages and debugging

## 🔗 Related Projects

### This Stack
- **📦 [libfreenect2-modern](https://github.com/cerealkiller2527/libfreenect2-modern)** - Foundation C++ library (install this first!)
- **🔧 [kinect-toolbox-py310](https://github.com/cerealkiller2527/kinect-toolbox-py310)** - High-level Python API built on this library

### Original Projects  
- **[OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2)** - Original core C++ library
- **[r9y9/pylibfreenect2](https://github.com/r9y9/pylibfreenect2)** - Original Python bindings (2.7-3.5)

### 🚀 Next Steps: High-Level API

For easier Kinect development, install the high-level wrapper:
```bash
pip install git+https://github.com/cerealkiller2527/kinect-toolbox-py310.git
```

This automatically installs pylibfreenect2-py310 as a dependency and provides a simple, OpenCV-like interface.

## License

MIT License - see [LICENSE.md](LICENSE.md) for details.

Original work by [Ryuichi Yamamoto](https://github.com/r9y9). Enhanced for Python 3.10+ compatibility.

---

**⚠️ Remember**: This is a complex package requiring significant setup. Budget time for troubleshooting, especially on Windows. Consider simpler alternatives if you just need basic Kinect functionality.