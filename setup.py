#!/usr/bin/env python3
"""
Setup script for HowYouSeeMe
Handles installation and environment setup
"""

import os
import sys
import subprocess
import platform
from pathlib import Path

def run_command(cmd, check=True, shell=False):
    """Run a command and handle errors"""
    print(f"Running: {cmd}")
    try:
        if isinstance(cmd, str) and not shell:
            cmd = cmd.split()
        result = subprocess.run(cmd, check=check, shell=shell, 
                              capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        return result.returncode == 0
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {e}")
        if e.stderr:
            print(f"Error: {e.stderr}")
        return False

def check_system_requirements():
    """Check system requirements"""
    print("Checking system requirements...")
    
    # Check Python version
    if sys.version_info < (3, 8):
        print("ERROR: Python 3.8+ required")
        return False
    
    # Check OS
    if platform.system() not in ['Linux', 'Darwin']:
        print("WARNING: This system is primarily tested on Linux/macOS")
    
    # Check for essential system packages
    essential_commands = ['cmake', 'pkg-config', 'git']
    for cmd in essential_commands:
        if not run_command(f"which {cmd}", check=False):
            print(f"ERROR: {cmd} not found. Please install build tools.")
            return False
    
    print("✓ System requirements check passed")
    return True

def setup_python_environment():
    """Set up Python virtual environment and dependencies"""
    print("Setting up Python environment...")
    
    # Create virtual environment if it doesn't exist
    venv_path = Path("venv")
    if not venv_path.exists():
        print("Creating virtual environment...")
        if not run_command([sys.executable, "-m", "venv", "venv"]):
            return False
    
    # Determine activation script
    if platform.system() == "Windows":
        activate_script = venv_path / "Scripts" / "activate"
        pip_path = venv_path / "Scripts" / "pip"
    else:
        activate_script = venv_path / "bin" / "activate"
        pip_path = venv_path / "bin" / "pip"
    
    # Install dependencies
    print("Installing Python dependencies...")
    if not run_command([str(pip_path), "install", "--upgrade", "pip"]):
        return False
    
    if not run_command([str(pip_path), "install", "-r", "requirements.txt"]):
        return False
    
    print("✓ Python environment setup completed")
    return True

def check_kinect_setup():
    """Check Kinect v2 setup"""
    print("Checking Kinect v2 setup...")
    
    # Check if libfreenect2 is built
    build_path = Path("libfreenect2/build")
    protonect_path = build_path / "bin" / "Protonect"
    
    if not protonect_path.exists():
        print("Kinect v2 (libfreenect2) not built yet.")
        print("Please run the Kinect setup:")
        print("  cd libfreenect2/build")
        print("  cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2")
        print("  make && make install")
        return False
    
    # Check if Protonect is in PATH
    if run_command("which Protonect", check=False):
        print("✓ Protonect found in PATH")
    else:
        print("Protonect not in PATH. You may want to run:")
        print(f"  sudo ln -sf {protonect_path.absolute()} /usr/local/bin/Protonect")
    
    # Check udev rules
    udev_rules = Path("/etc/udev/rules.d/90-kinect2.rules")
    if not udev_rules.exists():
        print("Kinect udev rules not installed. Run:")
        print("  sudo cp libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/")
        print("  # Then replug your Kinect")
        return False
    
    print("✓ Kinect setup appears complete")
    return True

def check_gpu_setup():
    """Check GPU setup for acceleration"""
    print("Checking GPU setup...")
    
    # Check NVIDIA GPU
    if run_command("nvidia-smi", check=False):
        print("✓ NVIDIA GPU detected")
        
        # Check CUDA
        try:
            import torch
            if torch.cuda.is_available():
                print(f"✓ CUDA available: {torch.cuda.get_device_name()}")
                print(f"  CUDA version: {torch.version.cuda}")
                return True
            else:
                print("⚠ CUDA not available in PyTorch")
        except ImportError:
            print("⚠ PyTorch not installed yet")
    else:
        print("⚠ No NVIDIA GPU detected, will use CPU")
    
    return True

def create_directories():
    """Create necessary directories"""
    print("Creating project directories...")
    
    directories = [
        "data/models",
        "data/cache", 
        "data/evidence",
        "logs",
        "config"
    ]
    
    for dir_path in directories:
        Path(dir_path).mkdir(parents=True, exist_ok=True)
    
    print("✓ Directories created")
    return True

def run_basic_test():
    """Run basic functionality test"""
    print("Running basic functionality test...")
    
    try:
        # Test imports
        sys.path.append("src")
        
        from perception.sensor_interface import KinectV2Interface
        from perception.slam.slam_interface import BasicSLAM
        from perception.detection.object_detector import YOLODetector
        
        print("✓ All modules import successfully")
        
        # Test basic initialization
        intrinsics = {'fx': 800, 'fy': 800, 'cx': 320, 'cy': 240}
        slam = BasicSLAM(intrinsics)
        print("✓ SLAM initialization successful")
        
        # Note: Don't test detector here as it downloads models
        print("✓ Basic functionality test passed")
        return True
        
    except Exception as e:
        print(f"✗ Basic test failed: {e}")
        return False

def main():
    """Main setup function"""
    print("HowYouSeeMe Setup Script")
    print("=" * 40)
    
    # Check system requirements
    if not check_system_requirements():
        print("Please install missing system requirements and try again.")
        return False
    
    # Set up Python environment
    if not setup_python_environment():
        print("Python environment setup failed.")
        return False
    
    # Create directories
    if not create_directories():
        print("Directory creation failed.")
        return False
    
    # Check GPU setup
    check_gpu_setup()
    
    # Check Kinect setup
    kinect_ok = check_kinect_setup()
    
    # Run basic test
    if not run_basic_test():
        print("Basic functionality test failed.")
        return False
    
    print("\n" + "=" * 40)
    print("Setup completed successfully!")
    print("=" * 40)
    
    print("\nNext steps:")
    print("1. Activate the virtual environment:")
    print("   source venv/bin/activate")
    
    if not kinect_ok:
        print("2. Set up Kinect v2 (see docs/kinect.md)")
    
    print("3. Run the integration test:")
    print("   python test_integration.py")
    
    print("\nFor detailed setup instructions, see docs/Getting_Started.md")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)