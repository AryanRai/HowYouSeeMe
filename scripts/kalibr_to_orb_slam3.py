#!/usr/bin/env python3
"""
Convert Kalibr calibration output to ORB-SLAM3 config format
Reads kalibr_output/camchain-imucam.yaml and updates Kinect2_RGBD_IMU.yaml
"""

import yaml
import sys
import os
import numpy as np

def load_kalibr_results(kalibr_file):
    """Load Kalibr calibration results"""
    with open(kalibr_file, 'r') as f:
        data = yaml.safe_load(f)
    return data

def extract_camera_intrinsics(kalibr_data):
    """Extract camera intrinsics from Kalibr output"""
    cam0 = kalibr_data['cam0']
    intrinsics = cam0['intrinsics']
    resolution = cam0['resolution']
    
    return {
        'fx': intrinsics[0],
        'fy': intrinsics[1],
        'cx': intrinsics[2],
        'cy': intrinsics[3],
        'width': resolution[0],
        'height': resolution[1]
    }

def extract_imu_to_cam_transform(kalibr_data):
    """Extract T_cam_imu transformation matrix"""
    # Kalibr provides T_cam_imu (transformation from IMU to camera)
    T_cam_imu = np.array(kalibr_data['cam0']['T_cam_imu'])
    return T_cam_imu

def update_orb_slam3_config(template_file, output_file, intrinsics, T_cam_imu):
    """Update ORB-SLAM3 config with calibration data"""
    with open(template_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Update camera intrinsics
    config['Camera.fx'] = float(intrinsics['fx'])
    config['Camera.fy'] = float(intrinsics['fy'])
    config['Camera.cx'] = float(intrinsics['cx'])
    config['Camera.cy'] = float(intrinsics['cy'])
    config['Camera.width'] = int(intrinsics['width'])
    config['Camera.height'] = int(intrinsics['height'])
    
    # Update IMU to camera transform
    # ORB-SLAM3 expects T_b_c1 (body/IMU to camera)
    T_flat = T_cam_imu.flatten().tolist()
    config['IMU.T_b_c1'] = {
        'rows': 4,
        'cols': 4,
        'dt': 'f',
        'data': [float(x) for x in T_flat]
    }
    
    # Write updated config
    with open(output_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"✓ Updated ORB-SLAM3 config: {output_file}")
    print(f"  Camera intrinsics: fx={intrinsics['fx']:.2f}, fy={intrinsics['fy']:.2f}")
    print(f"  Principal point: cx={intrinsics['cx']:.2f}, cy={intrinsics['cy']:.2f}")
    print(f"  Resolution: {intrinsics['width']}x{intrinsics['height']}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(script_dir)
    
    kalibr_file = os.path.join(workspace_root, 'kalibr_output', 'camchain-imucam.yaml')
    template_file = os.path.join(workspace_root, 'orb_slam3_configs', 'Kinect2_RGBD_IMU.yaml')
    output_file = template_file  # Update in place
    
    if not os.path.exists(kalibr_file):
        print(f"✗ Kalibr output not found: {kalibr_file}")
        print("  Run Phase 1 calibration first!")
        sys.exit(1)
    
    if not os.path.exists(template_file):
        print(f"✗ ORB-SLAM3 config template not found: {template_file}")
        sys.exit(1)
    
    print("Converting Kalibr calibration to ORB-SLAM3 format...")
    
    # Load Kalibr results
    kalibr_data = load_kalibr_results(kalibr_file)
    
    # Extract calibration data
    intrinsics = extract_camera_intrinsics(kalibr_data)
    T_cam_imu = extract_imu_to_cam_transform(kalibr_data)
    
    # Update ORB-SLAM3 config
    update_orb_slam3_config(template_file, output_file, intrinsics, T_cam_imu)
    
    print("\nNext step: Run setup_orb_slam3.sh to build ORB-SLAM3")

if __name__ == '__main__':
    main()
