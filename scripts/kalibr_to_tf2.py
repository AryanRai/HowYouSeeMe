#!/usr/bin/env python3
"""
Convert Kalibr camchain-imucam.yaml to TF2 static_transform_publisher arguments
"""

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: ./kalibr_to_tf2.py camchain-imucam.yaml")
        print("\nConverts Kalibr calibration results to TF2 static transform")
        sys.exit(1)

    # Load Kalibr results
    try:
        with open(sys.argv[1], 'r') as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: File '{sys.argv[1]}' not found")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error parsing YAML: {e}")
        sys.exit(1)

    # Extract T_cam_imu
    try:
        T_cam_imu = np.array(data['cam0']['T_cam_imu'])
    except KeyError:
        print("Error: Could not find cam0/T_cam_imu in YAML file")
        print("Make sure this is a Kalibr camchain-imucam output file")
        sys.exit(1)

    # Extract translation
    tx, ty, tz = T_cam_imu[0:3, 3]

    # Extract rotation and convert to quaternion
    R = T_cam_imu[0:3, 0:3]
    rot = Rotation.from_matrix(R)
    qx, qy, qz, qw = rot.as_quat()

    # Print results
    print("=" * 70)
    print("  Kalibr IMU-Camera Calibration → TF2 Transform")
    print("=" * 70)
    print(f"\nInput file: {sys.argv[1]}")
    print(f"\nT_cam_imu (4x4 matrix):")
    print(T_cam_imu)
    print(f"\nTranslation (meters):")
    print(f"  x: {tx:>10.6f}")
    print(f"  y: {ty:>10.6f}")
    print(f"  z: {tz:>10.6f}")
    print(f"\nRotation (quaternion):")
    print(f"  qx: {qx:>10.6f}")
    print(f"  qy: {qy:>10.6f}")
    print(f"  qz: {qz:>10.6f}")
    print(f"  qw: {qw:>10.6f}")
    
    # Sanity checks
    print(f"\n" + "=" * 70)
    print("  Sanity Checks")
    print("=" * 70)
    
    translation_magnitude = np.linalg.norm([tx, ty, tz])
    print(f"\nTranslation magnitude: {translation_magnitude:.3f} m")
    if translation_magnitude > 0.5:
        print("  ⚠️  WARNING: Large translation (>0.5m)")
        print("     Check if IMU and camera are rigidly mounted")
    else:
        print("  ✅ Translation magnitude looks reasonable")
    
    # Check if rotation is close to identity
    identity_diff = np.linalg.norm(R - np.eye(3))
    print(f"\nRotation difference from identity: {identity_diff:.3f}")
    if identity_diff > 1.0:
        print("  ⚠️  Large rotation from identity")
        print("     This is OK if IMU and camera have different orientations")
    else:
        print("  ✅ Rotation is close to identity")
    
    # Check for NaN or inf
    if np.any(np.isnan(T_cam_imu)) or np.any(np.isinf(T_cam_imu)):
        print("\n  ❌ ERROR: Transform contains NaN or inf values!")
        print("     Calibration failed - re-run with better data")
        sys.exit(1)
    else:
        print("\n  ✅ No NaN or inf values")
    
    # Print launch file snippet
    print(f"\n" + "=" * 70)
    print("  ROS2 Launch File Snippet")
    print("=" * 70)
    print(f"""
# Add this to ros2_ws/src/bluelily_bridge/launch/bluelily_imu.launch.py
# Replace the existing static_transform_publisher node

Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='imu_to_kinect_tf',
    arguments=[
        '{tx:.6f}', '{ty:.6f}', '{tz:.6f}',  # Translation (m)
        '{qx:.6f}', '{qy:.6f}', '{qz:.6f}', '{qw:.6f}',  # Rotation (quaternion)
        'imu_link',      # Parent frame
        'kinect2_link'   # Child frame
    ],
    output='screen'
),
""")
    
    # Print command line version
    print("=" * 70)
    print("  Command Line (for testing)")
    print("=" * 70)
    print(f"""
ros2 run tf2_ros static_transform_publisher \\
  {tx:.6f} {ty:.6f} {tz:.6f} \\
  {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} \\
  imu_link kinect2_link
""")
    
    print("=" * 70)
    print("\n✅ Conversion complete!")
    print("\nNext steps:")
    print("1. Update bluelily_imu.launch.py with the snippet above")
    print("2. Rebuild: cd ros2_ws && colcon build --packages-select bluelily_bridge")
    print("3. Source: source install/setup.bash")
    print("4. Test: ros2 launch bluelily_bridge bluelily_imu.launch.py")
    print("5. Verify: ros2 run tf2_ros tf2_echo imu_link kinect2_link")
    print("\n")

if __name__ == '__main__':
    main()
