# Git Setup for HowYouSeeMe ROS2 System

## 📋 Git Configuration

The HowYouSeeMe project uses comprehensive Git configuration to handle the diverse file types and build artifacts in a ROS2-based computer vision system.

### 🔧 Files Configured

1. **`.gitignore`** - Excludes build artifacts, models, and temporary files
2. **`.gitattributes`** - Handles line endings and file type detection

## 🚫 What's Ignored

### **Build Artifacts**
- `src/ros2_ws/build/` - ROS2 build directory
- `src/ros2_ws/install/` - ROS2 install directory  
- `src/ros2_ws/log/` - ROS2 build logs
- `__pycache__/` - Python cache files
- `*.pyc` - Python compiled files

### **External Dependencies**
- `libfreenect2/` - Kinect v2 library (install separately)
- `pylibfreenect2-py310/` - Python bindings (install separately)
- `src/ros2_ws/src/kinect2_ros2/` - External ROS2 package

### **AI Models & Data**
- `*.pt` - PyTorch models (downloaded automatically)
- `*.onnx` - ONNX models
- `data/cache/` - Cached data
- `data/models/` - Model storage
- `logs/` - System logs

### **User-Specific Files**
- `config/local_*.yaml` - Local configurations
- `~/.ros/camera_info/` - Kinect calibration files
- `.env` - Environment variables

## ✅ What's Tracked

### **Source Code**
- `src/ros2_ws/src/howyouseeme_ros2/` - Main ROS2 package
- `*.py` - Python source files
- `*.cpp`, `*.hpp` - C++ source files
- `CMakeLists.txt` - Build configuration

### **Configuration**
- `config/config.yaml` - System configuration
- `package.xml` - ROS2 package manifests
- `setup.py` - Python package setup

### **Documentation**
- `docs/` - Documentation files
- `README.md` - Project documentation
- `*.md` - Markdown files

### **Launch Files**
- `launch/*.launch.py` - ROS2 launch files
- `*.rviz` - RViz configurations

## 🔄 Git Workflow

### **Initial Setup**
```bash
# Clone the repository
git clone https://github.com/AryanRai/HowYouSeeMe.git
cd HowYouSeeMe

# Install external dependencies (not tracked)
./setup_kinect2_ros2.sh
```

### **Development Workflow**
```bash
# Check status
git status

# Add changes (only tracked files)
git add src/ros2_ws/src/howyouseeme_ros2/
git add docs/
git add config/

# Commit changes
git commit -m "feat: add YOLOv12 detection node"

# Push changes
git push origin main
```

### **What NOT to Commit**
```bash
# Don't add these directories
git add src/ros2_ws/build/     # ❌ Build artifacts
git add src/ros2_ws/install/   # ❌ Install directory
git add libfreenect2/          # ❌ External library
git add data/models/           # ❌ Downloaded models
git add logs/                  # ❌ Log files

# These are automatically ignored by .gitignore
```

## 📁 Repository Structure

### **Tracked Files**
```
HowYouSeeMe/
├── src/
│   └── ros2_ws/
│       └── src/
│           └── howyouseeme_ros2/    # ✅ Main package
├── config/                          # ✅ Configuration
├── docs/                           # ✅ Documentation  
├── README.md                       # ✅ Main docs
├── .gitignore                      # ✅ Git ignore rules
├── .gitattributes                  # ✅ Git attributes
└── setup_kinect2_ros2.sh          # ✅ Setup script
```

### **Ignored Files**
```
HowYouSeeMe/
├── src/ros2_ws/
│   ├── build/                      # ❌ Build artifacts
│   ├── install/                    # ❌ Install directory
│   └── log/                        # ❌ Build logs
├── libfreenect2/                   # ❌ External library
├── data/                           # ❌ Runtime data
├── logs/                           # ❌ System logs
└── __pycache__/                    # ❌ Python cache
```

## 🔧 Git Commands Reference

### **Check Repository Status**
```bash
# See what's changed
git status

# See what's ignored
git status --ignored

# Check file attributes
git check-attr -a filename
```

### **Handle Large Files**
```bash
# If using Git LFS for large models
git lfs track "*.pt"
git lfs track "*.bag"
git add .gitattributes
```

### **Clean Repository**
```bash
# Remove untracked files (be careful!)
git clean -fd

# Remove ignored files
git clean -fxd
```

## 🚀 Best Practices

### **Before Committing**
1. **Build the system** to ensure it works
2. **Run tests** with `python3 test_ros2_system.py`
3. **Check file sizes** - avoid committing large files
4. **Review changes** with `git diff`

### **Commit Messages**
Use conventional commit format:
```bash
git commit -m "feat: add YOLOv12 object detection"
git commit -m "fix: resolve kinect2_bridge startup issue"  
git commit -m "docs: update installation guide"
git commit -m "refactor: reorganize ROS2 package structure"
```

### **Branch Strategy**
```bash
# Create feature branch
git checkout -b feature/yolo-detection

# Work on feature
git add src/ros2_ws/src/howyouseeme_ros2/
git commit -m "feat: implement YOLOv12 detection node"

# Merge to main
git checkout main
git merge feature/yolo-detection
```

## 🔍 Troubleshooting

### **Large Repository Size**
```bash
# Check repository size
du -sh .git

# Find large files
git rev-list --objects --all | git cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' | awk '/^blob/ {print substr($0,6)}' | sort --numeric-sort --key=2 | tail -10
```

### **Accidentally Committed Large Files**
```bash
# Remove from history (use carefully!)
git filter-branch --force --index-filter 'git rm --cached --ignore-unmatch path/to/large/file' --prune-empty --tag-name-filter cat -- --all
```

### **Reset to Clean State**
```bash
# Reset to last commit (loses changes!)
git reset --hard HEAD

# Clean untracked files
git clean -fd
```

## 📚 Additional Resources

- [Git Documentation](https://git-scm.com/doc)
- [Git LFS](https://git-lfs.github.io/) - For large files
- [Conventional Commits](https://www.conventionalcommits.org/) - Commit message format
- [ROS2 Git Best Practices](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html)

---

**Remember**: Only commit source code and configuration. Let the build system handle dependencies and artifacts!