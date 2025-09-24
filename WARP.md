# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

HowYouSeeMe is a world perception system that combines computer vision models and systems (YOLO, SLAM, Segmentation, VLMs, etc.) into a unified World State Summarizer Interface using MCP (Model Context Protocol). The project is currently in early development stages.

The repository currently includes libfreenect2, a driver for Kinect for Windows v2 devices, which serves as the foundational sensor interface for depth and RGB data acquisition.

## Commands

### Building libfreenect2
```bash
cd libfreenect2/build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
```

### Running tests
```bash
# Run the main test program (Kinect sensor required)
cd libfreenect2/build
./bin/Protonect
```

### Building with different processing pipelines
```bash
# Build with OpenCL support (GPU acceleration)
cmake .. -DENABLE_OPENCL=ON

# Build with CUDA support (NVIDIA GPUs)
cmake .. -DENABLE_CUDA=ON

# Build with OpenGL support
cmake .. -DENABLE_OPENGL=ON

# Disable OpenGL (for systems without OpenGL 3.1)
cmake .. -DENABLE_OPENGL=OFF
```

### Running with specific pipelines
```bash
# Set environment variable to use specific processing pipeline
LIBFREENECT2_PIPELINE=cl ./bin/Protonect    # OpenCL
LIBFREENECT2_PIPELINE=cuda ./bin/Protonect  # CUDA
LIBFREENECT2_PIPELINE=opengl ./bin/Protonect # OpenGL
```

### Setting up device permissions (Linux)
```bash
sudo cp libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
# Replug the Kinect device after setting up udev rules
```

### Debug mode
```bash
# Enable USB debugging for troubleshooting
LIBUSB_DEBUG=3 ./bin/Protonect
```

## Architecture

### Current Structure
- **libfreenect2/**: Core Kinect v2 driver library
  - **src/**: Core implementation files
    - Packet processors for depth and RGB streams
    - USB communication and event handling
    - Multiple processing pipelines (CPU, OpenGL, OpenCL, CUDA)
  - **include/libfreenect2/**: Public API headers
  - **examples/**: Reference implementations (Protonect viewer)
  - **build/**: CMake build directory

### Key Components

#### Processing Pipelines
The libfreenect2 library supports multiple processing pipelines for depth data:
- **CPU Pipeline**: Pure CPU processing (slowest but most compatible)
- **OpenGL Pipeline**: GPU acceleration using OpenGL compute shaders
- **OpenCL Pipeline**: GPU acceleration using OpenCL (Intel/AMD GPUs)
- **CUDA Pipeline**: GPU acceleration using CUDA (NVIDIA GPUs)

#### Core Classes
- `Freenect2`: Main device interface
- `PacketPipeline`: Abstract base for processing pipelines
- `Frame` and `FrameMap`: Data structures for sensor data
- `Registration`: RGB-depth image alignment utilities

#### Data Flow
1. USB communication handles raw sensor data
2. Stream parsers decode packet data
3. Packet processors handle depth/RGB processing
4. Frame listeners receive processed data
5. Registration aligns RGB and depth frames

### Future Architecture (Planned)
Based on the project description, the system will expand to include:
- **YOLO Integration**: Object detection and classification
- **SLAM System**: Simultaneous localization and mapping
- **Segmentation**: Scene understanding and object segmentation  
- **VLM Integration**: Vision-language model processing
- **World State Summarizer**: Unified interface combining all perception modules
- **MCP Interface**: Model Context Protocol for external system integration

## Hardware Requirements

### Kinect v2 Requirements
- USB 3.0 controller (USB 2.0 not supported)
- Intel or NEC USB 3.0 controllers recommended
- Avoid ASMedia USB controllers
- For multiple Kinects: separate PCIe USB3 cards with x8/x16 slots

### GPU Requirements (Optional but Recommended)
- **OpenGL**: OpenGL 3.1+ support
- **OpenCL**: OpenCL 1.1+ (Intel integrated graphics, AMD, etc.)
- **CUDA**: CUDA-capable NVIDIA GPU (tested with 6.5, 7.5+)

## Environment Variables

- `LIBFREENECT2_PIPELINE`: Set processing pipeline (`cpu`, `opengl`, `cl`, `cuda`)
- `LIBUSB_DEBUG`: USB debug level (0-3, higher = more verbose)
- `TurboJPEG_ROOT`: Override TurboJPEG installation path
- `GLFW_ROOT`: Override GLFW installation path

## Troubleshooting

### Common Issues
- **USB 3.0 Issues**: Check `lsusb -t` and ensure proper USB 3.0 controller
- **Permission Issues**: Verify udev rules are installed and device is replugged
- **Build Issues**: Ensure all dependencies are installed (see kinect_setup.md)
- **Performance Issues**: Try different processing pipelines based on available hardware

### Debug Information
- Check hardware compatibility: `lspci` and `lsusb -t`  
- Monitor system logs: `dmesg | grep -i usb`
- Test OpenCL support: Install and run `clinfo`
- Verify CUDA installation: Check CUDA samples build and run

## Key Files
- `kinect_setup.md`: Detailed installation and setup instructions
- `libfreenect2/README.md`: Complete libfreenect2 documentation
- `libfreenect2/examples/Protonect.cpp`: Reference implementation
- `libfreenect2/include/libfreenect2/libfreenect2.hpp`: Main API