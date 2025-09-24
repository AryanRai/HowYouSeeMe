# Modern Kinect v2 Setup Guide for HowYouSeeMe

This guide will help you set up a modern Python 3.12 environment with `pylibfreenect2-py310` for high-performance Kinect v2 integration.

## 🎯 Overview

We're replacing your current Python 3.12 venv with an Anaconda environment that includes:
- **Python 3.12** (latest compatibility)
- **pylibfreenect2-py310** (modern Kinect bindings)
- **GPU acceleration** (CUDA/OpenCL/OpenGL pipelines)
- **All HowYouSeeMe dependencies**

## 📋 Prerequisites

### 1. Verify libfreenect2 Installation

First, make sure your existing libfreenect2 setup works:

```bash
# Test Protonect
Protonect --help

# Test with a few frames (requires Kinect connected)
Protonect -frames 10 -noviewer
```

If this doesn't work, you need to fix your libfreenect2 installation first.

### 2. Check System Dependencies

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install build-essential python3-dev pkg-config cmake git

# Check compiler
gcc --version
```

## 🚀 Step-by-Step Installation

### Step 1: Deactivate Current Environment

```bash
# Deactivate your current venv
deactivate

# Navigate to your project
cd ~/Documents/GitHub/HowYouSeeMe
```

### Step 2: Set Up Anaconda Environment

```bash
# Run the setup script
bash setup_anaconda_env.sh
```

This will:
- Remove any existing `howyouseeme` conda environment
- Create a new Python 3.12 environment
- Install all core dependencies
- Set up PyTorch with CUDA support (if available)

### Step 3: Activate New Environment

```bash
# Activate the new environment
conda activate howyouseeme

# Verify Python version
python --version  # Should show Python 3.12.x
```

### Step 4: Install Kinect Bindings

```bash
# Run the Kinect bindings installation
bash install_kinect_bindings.sh
```

This will:
- Detect your libfreenect2 installation
- Set required environment variables
- Install `pylibfreenect2-py310`
- Test the installation

### Step 5: Test the Installation

```bash
# Test the modern Kinect interface
python src/perception/modern_kinect_interface.py
```

Expected output:
```
Testing Modern Kinect v2 Interface with pylibfreenect2-py310
============================================================
✅ pylibfreenect2 is available
🚀 Available pipelines: ['cuda', 'opencl', 'opengl', 'cpu']
✅ Kinect v2 device detected
📷 RGB: 1920x1080
📷 Depth: 512x424
🎬 Testing streaming (10 seconds)...
✅ Streaming started
📸 Frame 10: RGB=True, Depth=True, IR=False
✅ Streaming test completed
📊 Received 95 frames in 10.0s (9.5 FPS)
🎉 Modern Kinect interface is working perfectly!
```

## 🔧 Environment Variables

The installation script will detect and set these automatically, but you can make them permanent:

```bash
# Add to ~/.bashrc (replace with your actual path)
echo 'export LIBFREENECT2_INSTALL_PREFIX=/home/aryan/freenect2' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/home/aryan/freenect2/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

# Reload
source ~/.bashrc
```

## 🧪 Testing Your Setup

### Basic Import Test

```python
import pylibfreenect2
print("✅ Import successful!")

# Check available pipelines
fn = pylibfreenect2.Freenect2()
print(f"Devices found: {fn.enumerateDevices()}")
```

### Performance Test

```python
from src.perception.modern_kinect_interface import ModernKinectInterface

kinect = ModernKinectInterface(preferred_pipeline="cuda")  # or "opencl"
print(f"Available pipelines: {kinect.get_available_pipelines()}")

if kinect.is_connected():
    print("✅ Kinect v2 ready for high-performance capture!")
```

### Integration Test

```bash
# Run the full integration test
python test_integration.py
```

## 🚨 Troubleshooting

### "No module named 'pylibfreenect2'"

```bash
# Make sure you're in the right environment
conda activate howyouseeme

# Reinstall if needed
pip install git+https://github.com/cerealkiller2527/pylibfreenect2-py310.git
```

### "No Kinect devices found"

```bash
# Check USB connection and permissions
lsusb | grep Microsoft

# Check udev rules
ls /etc/udev/rules.d/90-kinect2.rules

# If missing, install:
sudo cp libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
# Then replug your Kinect
```

### "No working pipeline found"

```bash
# Test libfreenect2 directly
Protonect

# Should show available pipelines like:
# [Info] [Freenect2Impl] enumerating devices...
# [Info] [Freenect2Impl] 1 devices found
# [Info] [CudaPacketPipeline] CUDA pipeline available
```

### Build Errors

```bash
# Install missing build dependencies
sudo apt-get install build-essential python3-dev cmake pkg-config

# Update pip and setuptools
pip install --upgrade pip setuptools wheel cython
```

## 🎉 Success Indicators

You'll know everything is working when:

1. ✅ `conda activate howyouseeme` works
2. ✅ `python -c "import pylibfreenect2"` succeeds
3. ✅ `python src/perception/modern_kinect_interface.py` shows streaming
4. ✅ Multiple GPU pipelines are available (CUDA, OpenCL, etc.)
5. ✅ Frame rates > 20 FPS with GPU acceleration

## 🔄 Switching Back (If Needed)

If you need to go back to your old setup:

```bash
# Deactivate conda
conda deactivate

# Activate your old venv
source venv/bin/activate
```

Your old environment is preserved and unchanged.

## 📚 Next Steps

Once your environment is working:

1. **Update your code** to use `ModernKinectInterface`
2. **Test GPU acceleration** with different pipelines
3. **Integrate with your SLAM and detection systems**
4. **Optimize for your specific use case**

## 🆘 Getting Help

If you run into issues:

1. Check the error messages carefully
2. Verify libfreenect2 works independently
3. Make sure you're using the correct conda environment
4. Check the [pylibfreenect2-py310 documentation](https://github.com/cerealkiller2527/pylibfreenect2-py310)

---

**Ready to get started? Run the setup scripts and let's get your modern Kinect environment working!**