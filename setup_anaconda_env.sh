#!/bin/bash
# HowYouSeeMe Anaconda Environment Setup Script
# Sets up a modern Python 3.12 environment with pylibfreenect2-py310

set -e  # Exit on any error

echo "🚀 Setting up HowYouSeeMe Anaconda Environment"
echo "================================================"

# Source conda
source ~/anaconda3/etc/profile.d/conda.sh

# Remove existing environment if it exists
echo "🧹 Cleaning up existing environment..."
conda env remove -n howyouseeme -y 2>/dev/null || true

# Create new environment with Python 3.12
echo "🐍 Creating new conda environment with Python 3.12..."
conda create -n howyouseeme python=3.12 -y

# Activate environment
echo "✅ Activating environment..."
conda activate howyouseeme

# Install conda packages first (faster and more reliable)
echo "📦 Installing conda packages..."
conda install -y \
    numpy>=1.21.0 \
    scipy>=1.7.0 \
    opencv>=4.5.0 \
    pillow>=8.0.0 \
    pyyaml>=6.0 \
    cython>=0.29.36 \
    cmake \
    pkg-config \
    pytest>=6.0.0 \
    black \
    flake8

# Install PyTorch with CUDA support (if available)
echo "🔥 Installing PyTorch with CUDA support..."
conda install -y pytorch torchvision pytorch-cuda=11.8 -c pytorch -c nvidia || \
conda install -y pytorch torchvision cpuonly -c pytorch

# Install pip packages
echo "📦 Installing pip packages..."
pip install --upgrade pip

# Install core dependencies
pip install \
    ultralytics>=8.0.0 \
    mediapipe>=0.8.0 \
    fastapi>=0.68.0 \
    uvicorn>=0.15.0 \
    websockets>=10.0 \
    redis>=4.0.0 \
    librosa>=0.8.0 \
    tqdm>=4.62.0 \
    python-dotenv>=0.19.0 \
    pytest-asyncio>=0.15.0

# Try to install pyaudio (might need system dependencies)
echo "🎵 Installing audio dependencies..."
pip install pyaudio>=0.2.11 || echo "⚠️  pyaudio installation failed - you may need to install portaudio system package"

echo "🎯 Environment setup complete!"
echo ""
echo "Next steps:"
echo "1. Activate environment: conda activate howyouseeme"
echo "2. Set up libfreenect2 environment variable"
echo "3. Install pylibfreenect2-py310"
echo ""
echo "Run this script with: bash setup_anaconda_env.sh"