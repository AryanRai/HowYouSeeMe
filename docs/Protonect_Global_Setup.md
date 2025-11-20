# Protonect Global Installation

## Overview
Protonect is now installed globally and can be run from anywhere on your system.

## Installation Details

### Binary Location
- **Wrapper Script**: `/usr/local/bin/Protonect`
- **Actual Binary**: `~/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/bin/Protonect`
- **Library Path**: `~/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib`

### Build Configuration
- **Compiler**: GCC 12 (required for CUDA 12.0 compatibility)
- **CUDA**: Enabled (NVIDIA GPU acceleration)
- **OpenCL**: Disabled (due to compatibility issues)
- **Source**: `libfreenect-new/libfreenect2-modern/`

## Usage

Run Protonect from any directory:

```bash
# Basic usage
Protonect

# Show help
Protonect --help

# Use CUDA acceleration
Protonect cuda

# Use CUDA KDE (Kinect Depth Engine)
Protonect cudakde

# CPU fallback
Protonect cpu

# Headless mode (no viewer)
Protonect -noviewer

# Process specific number of frames
Protonect -frames 100

# Disable specific streams
Protonect -norgb    # Disable RGB
Protonect -nodepth  # Disable depth
```

## Rebuilding

If you need to rebuild Protonect:

```bash
cd ~/Documents/GitHub/HowYouSeeMe/libfreenect-new/libfreenect2-modern/build

# Clean build directory
rm -rf *

# Configure with GCC 12 (required for CUDA)
cmake .. -DCMAKE_C_COMPILER=/usr/bin/gcc-12 \
         -DCMAKE_CXX_COMPILER=/usr/bin/g++-12 \
         -DENABLE_OPENCL=OFF

# Build
make -j$(nproc)

# Copy to installation location
cp bin/Protonect ~/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/bin/Protonect
```

## Wrapper Script

The wrapper script at `/usr/local/bin/Protonect` automatically sets the library path:

```bash
#!/bin/bash
# Protonect wrapper script with library path
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH
exec /home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/bin/Protonect "$@"
```

## Troubleshooting

### Command not found
If `Protonect` is not found, check:
```bash
which Protonect
ls -la /usr/local/bin/Protonect
```

### Library errors
If you get library loading errors:
```bash
ldd ~/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/bin/Protonect
```

### Permissions
Ensure the wrapper is executable:
```bash
sudo chmod +x /usr/local/bin/Protonect
```

## Version Info
- **libfreenect2**: 0.2.0
- **CUDA**: 12.0
- **GCC**: 12.4.0
