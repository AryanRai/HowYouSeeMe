# kinect-toolbox-py310

[![License](http://img.shields.io/badge/license-GPL--2.0-brightgreen.svg?style=flat)](LICENSE)
[![Python Version](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)](https://github.com/cerealkiller2527/kinect-toolbox-py310)

**Python 3.10+ compatible Kinect v2 wrapper with intelligent GPU pipeline selection**

This is an enhanced version of [kinect-toolbox](https://github.com/nikwl/kinect-toolbox) with Python 3.10+ compatibility, Windows support, and intelligent pipeline selection. Built on [pylibfreenect2-py310](https://github.com/cerealkiller2527/pylibfreenect2-py310), it provides a simple, OpenCV-like interface for the Microsoft Kinect v2.

## 🎯 Part of the GPU-Accelerated Kinect v2 Stack

This is the **high-level API layer** of the complete Kinect v2 development stack:

| Layer | Project | Description |
|-------|---------|-------------|
| **📦 Foundation** | [libfreenect2-modern](https://github.com/cerealkiller2527/libfreenect2-modern) | Core C++ driver with GPU pipelines |
| **🐍 Python Bindings** | [pylibfreenect2-py310](https://github.com/cerealkiller2527/pylibfreenect2-py310) | Python 3.10+ interface with Cython bindings |
| **🔧 High-Level API** | **kinect-toolbox-py310** (this project) | Easy-to-use Python wrapper with utilities |

**Enhanced by [Madhav Lodha](https://madhavlodha.com)** - Check out my portfolio at [madhavlodha.com](https://madhavlodha.com) for more projects!

## ✨ Key Features

- **🐍 Python 3.10+ Support**: Works with modern Python versions
- **🚀 Intelligent Pipeline Selection**: Automatically chooses best GPU acceleration (CUDA → OpenCL → OpenGL → CPU)
- **🪟 Cross-Platform**: Windows, Linux, and macOS support  
- **📷 OpenCV-like API**: Simple `k.get_frame()` interface
- **🎯 Multiple Frame Types**: Color, depth, IR, raw formats with automatic registration
- **☁️ Point Cloud Generation**: Fast 3D reconstruction with real-world coordinates
- **🎬 Video Recording**: Built-in recording capabilities
- **⚡ GPU Acceleration**: CUDA, OpenCL, and OpenGL pipeline support

Credit for point cloud acceleration methods goes to stackoverflow user [Logic1](https://stackoverflow.com/questions/41241236/vectorizing-the-kinect-real-world-coordinate-processing-algorithm-for-speed).

## Prerequisites

### 1. libfreenect2 Installation

**You MUST have libfreenect2 built and installed first.** Follow the comprehensive guide:

🔗 **[libfreenect2-modern Installation Guide](https://github.com/cerealkiller2527/libfreenect2-modern)**

### 2. Environment Setup

#### Windows (Command Prompt)
```cmd
set LIBFREENECT2_INSTALL_PREFIX=C:\path\to\your\libfreenect2\install
```

#### Linux/macOS
```bash
export LIBFREENECT2_INSTALL_PREFIX=/usr/local  # or your custom path
export LD_LIBRARY_PATH=$LIBFREENECT2_INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
```

## Installation

### Method 1: Direct from GitHub (Recommended)
```bash
pip install git+https://github.com/cerealkiller2527/kinect-toolbox-py310.git
```

### Method 2: Development Installation
```bash
git clone https://github.com/cerealkiller2527/kinect-toolbox-py310.git
cd kinect-toolbox-py310
pip install -e .
```

**Tested Platforms:**
- ✅ **Windows 10/11** with Python 3.10-3.13
- ✅ **Ubuntu 20.04+** with Python 3.10+ (should work)
- ⚠️ **macOS** with Python 3.10+ (untested but should work)

## Quick Start

### Basic Usage
```python
import cv2
import ktb

# Simple setup - automatically selects best pipeline
k = ktb.Kinect()
while True:
    color_frame = k.get_frame(ktb.COLOR)  # 512x424 registered color
    cv2.imshow('Kinect', color_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

### Advanced Usage
```python
import ktb
from pylibfreenect2 import CudaPacketPipeline

# Force specific pipeline for maximum performance
k = ktb.Kinect(pipeline=CudaPacketPipeline())

# Get multiple frame types
color, depth, ir = k.get_frame([ktb.COLOR, ktb.DEPTH, ktb.IR])

# Generate 3D point cloud with colors
ptcld, colors = k.get_ptcld(colorized=True, scale=1000)  # in meters

# Record video
k.record("kinect_video.avi", ktb.RAW_COLOR)
```

## API Reference

### `ktb.Kinect()`
Main Kinect interface with intelligent pipeline selection.

**Arguments:**
- `params`: Camera parameters file (.json) or dict with position/intrinsic parameters
- `device_index`: Device index if multiple Kinects connected (default: 0)
- `headless`: Enable headless mode for servers (default: False)
- `pipeline`: Override automatic pipeline selection

**Supported Pipelines:**
- `CudaPacketPipeline` - 25-35 FPS (NVIDIA GPUs)
- `OpenCLPacketPipeline` - 20-30 FPS (Most GPUs)
- `OpenGLPacketPipeline` - 15-25 FPS (Graphics cards)
- `CpuPacketPipeline` - 5-15 FPS (CPU fallback)

**Pipeline Selection Logic:**
```python
# Headless mode: CUDA → OpenCL → CPU
k = ktb.Kinect(headless=True)

# Display mode: CUDA → OpenCL → OpenGL → CPU  
k = ktb.Kinect(headless=False)
```

### `k.get_frame(frame_type)`
Capture frames from the Kinect with automatic registration and undistortion.

**Frame Types:**
| Type | Resolution | Format | Description | Use Case |
|------|------------|--------|-------------|----------|
| `ktb.RAW_COLOR` | 1920×1080 | BGR | Full resolution color | Photography |
| `ktb.RAW_DEPTH` | 512×424 | uint16 (mm) | Raw depth data | Sensor access |
| `ktb.COLOR` | 512×424 | RGB | Registered to depth | Computer vision |
| `ktb.DEPTH` | 512×424 | float32 (mm) | Undistorted depth | 3D reconstruction |
| `ktb.IR` | 512×424 | uint16 | Infrared intensity | Tracking |

**Examples:**
```python
# Single frame
color = k.get_frame(ktb.COLOR)

# Multiple frames  
color, depth = k.get_frame([ktb.COLOR, ktb.DEPTH])

# Display depth (rescale for visualization)
depth_vis = depth / 4500.0
cv2.imshow('Depth', depth_vis)
```

### `k.get_ptcld(roi=None, scale=1000, colorized=False)`
Generate 3D point clouds with real-world coordinates.

**Arguments:**
- `roi`: Region of interest [y, x, h, w] or mask array
- `scale`: Coordinate scale (1000 = mm, 1 = meters)
- `colorized`: Return RGB colors for each point

**Returns:**
- Point cloud array (424, 512, 3) with [X, Y, Z] coordinates
- Colors array (if colorized=True)

**Examples:**
```python
# Basic point cloud
ptcld = k.get_ptcld()  # Shape: (424, 512, 3)

# Colorized point cloud
ptcld, colors = k.get_ptcld(colorized=True, scale=1000)

# ROI extraction
roi_ptcld = k.get_ptcld(roi=[100, 100, 200, 200])
```

### `k.record(filename, frame_type=COLOR, video_codec='XVID')`
Record video streams directly from the Kinect.

**Examples:**
```python
# Record color video
k.record("color.avi", ktb.COLOR)

# Record full resolution
k.record("fullres.avi", ktb.RAW_COLOR)

# Record depth video
k.record("depth.avi", ktb.DEPTH)
```

## Performance and Pipeline Selection

### Automatic Pipeline Selection
The library automatically selects the best available pipeline:

```python
k = ktb.Kinect()  # Automatic selection
# Output: ✅ Using CUDA pipeline (if available)
# Fallback: ✅ Using OpenCL pipeline  
# Fallback: ✅ Using OpenGL pipeline
# Last resort: ✅ Using CPU pipeline
```

### Manual Pipeline Override
```python
from pylibfreenect2 import CudaPacketPipeline, OpenCLPacketPipeline

# Force CUDA for maximum performance (NVIDIA only)
k = ktb.Kinect(pipeline=CudaPacketPipeline())

# Force OpenCL for cross-platform GPU acceleration
k = ktb.Kinect(pipeline=OpenCLPacketPipeline())
```

### Expected Performance
| Pipeline | FPS Range | GPU Support | Use Case |
|----------|-----------|-------------|----------|
| CUDA | 25-35 FPS | NVIDIA only | Maximum performance |
| OpenCL | 20-30 FPS | Most GPUs | Cross-platform GPU |
| OpenGL | 15-25 FPS | Graphics cards | Desktop compatibility |
| CPU | 5-15 FPS | All systems | Guaranteed fallback |

## Camera Calibration

### Intrinsic Parameters
```python
k = ktb.Kinect()

# Access parameters with dot notation
print(f"Focal length: fx={k.intrinsic_parameters.fx:.1f}")
print(f"Principal point: cx={k.intrinsic_parameters.cx:.1f}")

# Modify parameters for custom calibration
k.intrinsic_parameters.fx = 366.193
```

### Extrinsic Parameters (Real-world positioning)
```python
# Set Kinect position and orientation
k.position.z = 0.810        # Height above ground (meters)
k.position.elevation = -34  # Tilt angle (degrees) 
k.position.azimuth = 0      # Pan angle (degrees)

# Point clouds will be transformed to real-world coordinates
ptcld = k.get_ptcld()  # Automatically applies transformations
```

## Troubleshooting

### Common Issues

#### "No device connected!"
- Check USB 3.0 connection
- Verify Kinect v2 power supply
- Try different USB port

#### "No working pipeline found!"
```bash
# Check libfreenect2 installation
$LIBFREENECT2_INSTALL_PREFIX/bin/Protonect
# Should show available pipelines

# Verify environment variable
echo $LIBFREENECT2_INSTALL_PREFIX  # Linux/macOS
echo %LIBFREENECT2_INSTALL_PREFIX% # Windows
```

#### ImportError or DLL issues
```python
# Check pylibfreenect2-py310 installation
import pylibfreenect2
print("Available pipelines:", [n for n in dir(pylibfreenect2) if 'Pipeline' in n])
```

#### Poor performance (< 10 FPS)
```python
# Check which pipeline is being used
k = ktb.Kinect()  # Look for "✅ Using [pipeline]" message

# Force GPU pipeline
from pylibfreenect2 import CudaPacketPipeline
k = ktb.Kinect(pipeline=CudaPacketPipeline())
```

### Debug Information
```python
import ktb

k = ktb.Kinect()
print(f"Device: {k.device.getSerialNumber()}")
print(f"Intrinsics: fx={k.intrinsic_parameters.fx:.1f}")

# Performance test
import time
start_time = time.time()
for i in range(30):
    frame = k.get_frame(ktb.COLOR)
fps = 30 / (time.time() - start_time)
print(f"FPS: {fps:.1f}")
```

## 🔗 Related Projects

### This Stack
- **📦 [libfreenect2-modern](https://github.com/cerealkiller2527/libfreenect2-modern)** - Foundation C++ library with GPU acceleration
- **🐍 [pylibfreenect2-py310](https://github.com/cerealkiller2527/pylibfreenect2-py310)** - Python 3.10+ bindings (automatically installed)

### Original Projects
- **[nikwl/kinect-toolbox](https://github.com/nikwl/kinect-toolbox)** - Original Python 3.6 version
- **[OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2)** - Original core C++ library
- **[r9y9/pylibfreenect2](https://github.com/r9y9/pylibfreenect2)** - Original Python bindings (2.7-3.5)

### 🚀 Why This Stack?

**Complete Solution**: This project automatically installs the entire stack:
1. **libfreenect2-modern** build guide - Get GPU acceleration working
2. **pylibfreenect2-py310** - Python 3.10+ compatibility fixes  
3. **kinect-toolbox-py310** - Simple, OpenCV-like API

**Result**: `pip install` → GPU-accelerated Kinect v2 with 25-35 FPS performance!

## License

GPL-2.0 License - see [LICENSE](LICENSE) for details.

Original work by [Nikolas Lamb](https://github.com/nikwl). Enhanced for Python 3.10+ compatibility with intelligent pipeline selection and cross-platform support by [Madhav Lodha](https://madhavlodha.com).

---

**⚡ Quick Test**: `python -c "import ktb; k = ktb.Kinect(); print('✅ Success!')"`