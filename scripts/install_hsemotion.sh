#!/bin/bash
# Install HSEmotion - High-Speed Emotion Recognition
# Better alternative to FER with faster inference and better accuracy

echo "========================================"
echo "Installing HSEmotion"
echo "========================================"

# Activate conda environment
echo "Activating howyouseeme environment..."
source ~/anaconda3/etc/profile.d/conda.sh
conda activate howyouseeme

# Install HSEmotion
echo "Installing HSEmotion..."
pip install hsemotion

# Install additional dependencies
echo "Installing timm (PyTorch Image Models)..."
pip install timm

echo ""
echo "✅ HSEmotion installation complete!"
echo ""
echo "Test with:"
echo "python3 -c 'import hsemotion; print(hsemotion.__version__)'"
