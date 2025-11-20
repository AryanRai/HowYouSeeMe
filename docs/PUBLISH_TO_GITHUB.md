# How to Publish Your Fork to GitHub

Your kinect2_ros2_cuda fork is ready to publish! Here's how:

## Current Status

✅ **Proper Fork Created**
- Cloned from: `https://github.com/krepa098/kinect2_ros2`
- Upstream remote: `upstream` (points to krepa098/kinect2_ros2)
- Branch: `cuda-acceleration`
- Commit: `1fe39a2`
- Changes: CPU registration fixes, library linking fixes, documentation

## Method 1: Fork via GitHub (Recommended)

This creates a proper fork relationship on GitHub.

### Step 1: Fork on GitHub
1. Go to https://github.com/krepa098/kinect2_ros2
2. Click the **"Fork"** button (top right)
3. Select your account (AryanRai)
4. Optionally rename to `kinect2_ros2_cuda`
5. Click "Create fork"

### Step 2: Add Your Fork as Remote
```bash
cd ~/Documents/GitHub/HowYouSeeMe/kinect2_ros2_cuda

# Add your fork as origin
git remote add origin https://github.com/AryanRai/kinect2_ros2_cuda.git

# Verify remotes
git remote -v
# Should show:
# origin    https://github.com/AryanRai/kinect2_ros2_cuda.git (fetch)
# origin    https://github.com/AryanRai/kinect2_ros2_cuda.git (push)
# upstream  https://github.com/krepa098/kinect2_ros2.git (fetch)
# upstream  https://github.com/krepa098/kinect2_ros2.git (push)
```

### Step 3: Push Your Changes
```bash
# Push your cuda-acceleration branch
git push -u origin cuda-acceleration

# Optionally, also push main branch
git checkout main
git push -u origin main
```

### Step 4: Set Default Branch (Optional)
On GitHub:
1. Go to your fork: https://github.com/AryanRai/kinect2_ros2_cuda
2. Settings → Branches
3. Change default branch to `cuda-acceleration`

## Method 2: Create New Repository

If you prefer a standalone repository (not shown as fork on GitHub):

### Step 1: Create Repository
1. Go to https://github.com/new
2. Repository name: `kinect2_ros2_cuda`
3. Description: "Kinect2 ROS2 bridge with CPU registration fixes and CUDA support - Fork of krepa098/kinect2_ros2"
4. Public
5. **Don't** initialize with README, .gitignore, or license
6. Click "Create repository"

### Step 2: Push
```bash
cd ~/Documents/GitHub/HowYouSeeMe/kinect2_ros2_cuda

# Add remote
git remote add origin https://github.com/AryanRai/kinect2_ros2_cuda.git

# Push
git push -u origin cuda-acceleration

# Also push main if you want
git checkout main
git push -u origin main
```

## After Publishing

### Update README
Add a badge to show it's a fork:
```markdown
> **Fork of [krepa098/kinect2_ros2](https://github.com/krepa098/kinect2_ros2)**
```

### Create Release
1. Go to your repo → Releases → "Create a new release"
2. Tag: `v1.0.0`
3. Title: "v1.0.0 - CPU Registration Fixes"
4. Description: Copy from CHANGELOG.md
5. Publish release

### Update HowYouSeeMe Project
```bash
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src

# Backup current
mv kinect2_ros2 kinect2_ros2_backup

# Clone your fork
git clone https://github.com/AryanRai/kinect2_ros2_cuda.git kinect2_ros2
cd kinect2_ros2
git checkout cuda-acceleration

# Rebuild
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws
colcon build --packages-select kinect2_registration kinect2_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -Dfreenect2_DIR=$HOME/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib/cmake/freenect2
```

## Keeping Fork Updated

### Pull Updates from Upstream
```bash
cd ~/Documents/GitHub/HowYouSeeMe/kinect2_ros2_cuda

# Fetch upstream changes
git fetch upstream

# Merge into your branch
git checkout cuda-acceleration
git merge upstream/master

# Push to your fork
git push origin cuda-acceleration
```

### Contributing Back to Upstream

If you want to contribute your fixes back:

1. **Create Pull Request**:
   - Go to https://github.com/krepa098/kinect2_ros2
   - Click "Pull requests" → "New pull request"
   - Click "compare across forks"
   - Base: `krepa098/kinect2_ros2:master`
   - Head: `AryanRai/kinect2_ros2_cuda:cuda-acceleration`
   - Create PR with description of fixes

2. **PR Description Template**:
   ```markdown
   ## Fixes Critical Bugs in kinect2_ros2
   
   This PR fixes three critical issues preventing the bridge from working:
   
   1. **CPU Registration Not Available**
      - Added DEPTH_REG_CPU definition to bridge CMakeLists.txt
      - Fixes: "CPU registration is not available!" error
   
   2. **Library Linking Errors**
      - Fixed libfreenect2 linking with proper directories
      - Fixes: "cannot find -lfreenect2" error
   
   3. **GLX BadAccess Errors**
      - Changed default to CPU mode
      - Fixes: OpenGL conflicts on some systems
   
   **Tested on**: ROS2 Jazzy, Ubuntu 24.04, Kinect v2
   **Performance**: ~15Hz depth, ~30Hz color
   
   See CHANGELOG.md for detailed changes.
   ```

## Verification

After publishing, verify:

```bash
# Clone your fork fresh
cd /tmp
git clone https://github.com/AryanRai/kinect2_ros2_cuda.git
cd kinect2_ros2_cuda

# Check branches
git branch -a

# Check remotes
git remote -v

# Check commit history
git log --oneline -5
```

## Repository Settings

Recommended settings on GitHub:

### General
- ✅ Issues enabled
- ✅ Discussions disabled (use Issues)
- ✅ Projects disabled
- ✅ Wiki disabled

### Branches
- Default branch: `cuda-acceleration`
- Branch protection: Optional for main

### Topics (Tags)
Add these topics to your repo:
- `ros2`
- `kinect`
- `kinect-v2`
- `cuda`
- `computer-vision`
- `robotics`
- `slam`
- `depth-camera`

## Success Checklist

- [ ] Fork created on GitHub
- [ ] Remote added to local repo
- [ ] Changes pushed to GitHub
- [ ] README visible on GitHub
- [ ] CHANGELOG visible on GitHub
- [ ] Can clone fresh copy
- [ ] Upstream relationship visible (if using fork method)
- [ ] Topics/tags added
- [ ] Release created (optional)
- [ ] HowYouSeeMe updated to use your fork

## Need Help?

If you encounter issues:
1. Check `git remote -v` shows correct URLs
2. Check `git status` shows clean working tree
3. Check `git log` shows your commit
4. Try `git push -v` for verbose output
5. Check GitHub authentication: `gh auth status`

## Next Steps After Publishing

1. **Test Installation**: Follow QUICKSTART.md on a fresh system
2. **Enable CUDA**: Start working on CUDA acceleration
3. **Benchmark**: Compare CPU vs CUDA performance
4. **Document**: Add performance comparisons
5. **Share**: Post on ROS Discourse, Reddit r/ROS
