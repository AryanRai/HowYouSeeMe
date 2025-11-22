#!/bin/bash
# Install InsightFace for face recognition

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Installing InsightFace${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Activate conda environment
echo -e "${YELLOW}Activating howyouseeme environment...${NC}"
source ~/anaconda3/bin/activate howyouseeme

# Install InsightFace
echo -e "${YELLOW}Installing InsightFace...${NC}"
pip install insightface

# Install ONNX Runtime GPU
echo -e "${YELLOW}Installing ONNX Runtime GPU...${NC}"
pip install onnxruntime-gpu

# Create face database directory
echo -e "${YELLOW}Creating face database directory...${NC}"
mkdir -p data/faces

echo ""
echo -e "${GREEN}✅ InsightFace installation complete!${NC}"
echo ""
echo "Test with:"
echo "  python3 -c 'import insightface; print(insightface.__version__)'"
echo ""
