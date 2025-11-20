# Setup kinect2_ros2_cuda as Git Submodule

## Step 1: Push Your Fork to GitHub

First, you need to create the repository on GitHub and push your fork.

### Option A: Fork on GitHub (Recommended)

1. **Go to**: https://github.com/krepa098/kinect2_ros2
2. **Click**: "Fork" button (top right)
3. **Rename** (optional): Change name to `kinect2_ros2_cuda`
4. **Create fork**

Then push your changes:
```bash
cd ~/Documents/GitHub/HowYouSeeMe/kinect2_ros2_cuda

# Add your fork as remote
git remote add origin https://github.com/AryanRai/kinect2_ros2_cuda.git

# Push your branch
git push -u origin cuda-acceleration

# Also push main
git checkout main
git push -u origin main

# Switch back
git checkout cuda-acceleration
```

### Option B: Create New Repository

1. **Go to**: https://github.com/new
2. **Name**: `kinect2_ros2_cuda`
3. **Description**: "Kinect2 ROS2 bridge with CPU registration fixes and CUDA support"
4. **Public**
5. **Don't** initialize with README
6. **Create repository**

Then push:
```bash
cd ~/Documents/GitHub/HowYouSeeMe/kinect2_ros2_cuda
git remote add origin https://github.com/AryanRai/kinect2_ros2_cuda.git
git push -u origin cuda-acceleration
git checkout main
git push -u origin main
git checkout cuda-acceleration
```

## Step 2: Add as Submodule

Once your fork is on GitHub, add it as a submodule:

```bash
cd ~/Documents/GitHub/HowYouSeeMe

# Add submodule
git submodule add -b cuda-acceleration \
  https://github.com/AryanRai/kinect2_ros2_cuda.git \
  ros2_ws/src/kinect2_ros2_cuda

# Initialize and update
git submodule init
git submodule update

# Commit the submodule
git add .gitmodules ros2_ws/src/kinect2_ros2_cuda
git commit -m "Add kinect2_ros2_cuda as submodule

- Fork of krepa098/kinect2_ros2 with CPU registration fixes
- Branch: cuda-acceleration
- Fixes: CPU registration, library linking, GLX errors
- Ready for CUDA acceleration"
```

## Step 3: Rebuild

After adding as submodule:

```bash
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws

# Clean build
rm -rf build/kinect2_* install/kinect2_*

# Rebuild
source /opt/ros/jazzy/setup.bash
colcon build --packages-select kinect2_registration kinect2_bridge \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -Dfreenect2_DIR=$HOME/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib/cmake/freenect2

# Source
source install/setup.bash
```

## Step 4: Verify

```bash
# Verify submodule
git submodule status

# Should show:
# <commit-hash> ros2_ws/src/kinect2_ros2_cuda (heads/cuda-acceleration)

# Test
./verify_kinect2_fork.sh
./test_kinect2_ros2.sh
```

## Benefits of Submodule

### ✅ Advantages
- **Version Control**: Track specific commit of fork
- **Easy Updates**: `git submodule update --remote`
- **Clean Separation**: Fork development separate from main project
- **Reproducible**: Others can clone with exact same fork version

### 📝 Usage

**Clone project with submodules:**
```bash
git clone --recursive https://github.com/AryanRai/HowYouSeeMe.git
```

**Update submodule to latest:**
```bash
cd ~/Documents/GitHub/HowYouSeeMe
git submodule update --remote ros2_ws/src/kinect2_ros2_cuda
git add ros2_ws/src/kinect2_ros2_cuda
git commit -m "Update kinect2_ros2_cuda submodule"
```

**Work on submodule:**
```bash
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_ros2_cuda

# Make changes
git checkout cuda-acceleration
# ... edit files ...
git add .
git commit -m "Your changes"
git push origin cuda-acceleration

# Update main project to track new commit
cd ~/Documents/GitHub/HowYouSeeMe
git add ros2_ws/src/kinect2_ros2_cuda
git commit -m "Update kinect2_ros2_cuda to latest"
```

## Alternative: Keep as Regular Directory

If you prefer NOT to use submodules:

```bash
# Just copy the fork back
cp -r ~/Documents/GitHub/HowYouSeeMe/kinect2_ros2_cuda \
      ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/

# Add to .gitignore
echo "ros2_ws/src/kinect2_ros2_cuda/" >> .gitignore

# Rebuild
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws
colcon build --packages-select kinect2_registration kinect2_bridge
```

**Pros**: Simpler, no submodule complexity
**Cons**: Fork changes not tracked in main repo

## Recommended Approach

**Use submodule if:**
- ✅ You want to track fork version in main project
- ✅ You plan to update fork frequently
- ✅ Others will clone your project

**Don't use submodule if:**
- ❌ You find submodules confusing
- ❌ Fork is stable and won't change
- ❌ You prefer simplicity

## Current Status

- ✅ Fork created: `kinect2_ros2_cuda/`
- ✅ Fork working and verified
- ⏳ Fork needs to be pushed to GitHub
- ⏳ Then can be added as submodule

## Next Steps

1. **Push fork to GitHub** (see Step 1)
2. **Add as submodule** (see Step 2)
3. **Rebuild** (see Step 3)
4. **Verify** (see Step 4)
5. **Commit** submodule to main project

## Questions?

**Q: What if I haven't pushed to GitHub yet?**
A: You need to push first. Follow Step 1.

**Q: Can I use local path for submodule?**
A: Not recommended. Use GitHub URL for portability.

**Q: What if submodule breaks?**
A: You can always remove it and copy directory back.

**Q: How do I remove submodule?**
```bash
git submodule deinit ros2_ws/src/kinect2_ros2_cuda
git rm ros2_ws/src/kinect2_ros2_cuda
rm -rf .git/modules/ros2_ws/src/kinect2_ros2_cuda
```

## Documentation

- Git Submodules: https://git-scm.com/book/en/v2/Git-Tools-Submodules
- Your fork: `kinect2_ros2_cuda/README.md`
- Main project: `README.md`
