# Kinect v2 Setup and Usage

This document provides essential information for setting up and using the Kinect v2 sensor with libfreenect2.

## Quick Setup

### Building libfreenect2
```bash
cd libfreenect2/build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
```

### Setting up device permissions (Linux)
```bash
sudo cp libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
# Replug the Kinect device after setting up udev rules
```

## Running Tests

### Basic test (Kinect sensor required)
```bash
cd libfreenect2/build
./bin/Protonect
```

### Debug mode
```bash
# Enable USB debugging for troubleshooting
LIBUSB_DEBUG=3 ./bin/Protonect
```

## Processing Pipelines

The libfreenect2 library supports multiple processing pipelines for depth data processing:

### Available Pipelines
- **CPU Pipeline**: Pure CPU processing (slowest but most compatible)
- **OpenGL Pipeline**: GPU acceleration using OpenGL compute shaders
- **OpenCL Pipeline**: GPU acceleration using OpenCL (Intel/AMD GPUs)
- **CUDA Pipeline**: GPU acceleration using CUDA (NVIDIA GPUs)

### Building with GPU Support
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

### Running with Specific Pipelines
```bash
# Set environment variable to use specific processing pipeline
LIBFREENECT2_PIPELINE=cl ./bin/Protonect    # OpenCL
LIBFREENECT2_PIPELINE=cuda ./bin/Protonect  # CUDA
LIBFREENECT2_PIPELINE=opengl ./bin/Protonect # OpenGL
LIBFREENECT2_PIPELINE=cpu ./bin/Protonect   # CPU (default)
```

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

## Data Flow Architecture

1. **USB Communication**: Handles raw sensor data from Kinect v2
2. **Stream Parsers**: Decode packet data (RGB and depth streams)
3. **Packet Processors**: Handle depth/RGB processing using selected pipeline
4. **Frame Listeners**: Receive processed data
5. **Registration**: Align RGB and depth frames for unified output

## Core Classes

- `Freenect2`: Main device interface for sensor control
- `PacketPipeline`: Abstract base for processing pipelines
- `Frame` and `FrameMap`: Data structures for sensor data
- `Registration`: RGB-depth image alignment utilities

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

## Integration Notes

The Kinect v2 sensor serves as the primary input device for the HowYouSeeMe world perception system, providing:

- **RGB Stream**: Color image data for object detection and recognition
- **Depth Stream**: 3D spatial information for SLAM and spatial understanding
- **Aligned Data**: Registered RGB-depth frames for comprehensive scene analysis

This sensor data forms the foundation for the computer vision pipeline that will include YOLO object detection, SLAM mapping, segmentation, and VLM processing.

## References

- **Full Setup Guide**: `docs/kinect_setup.md` - Complete installation instructions
- **API Documentation**: `libfreenect2/README.md` - Complete libfreenect2 documentation  
- **Reference Implementation**: `libfreenect2/examples/Protonect.cpp`
- **Main API**: `libfreenect2/include/libfreenect2/libfreenect2.hpp`