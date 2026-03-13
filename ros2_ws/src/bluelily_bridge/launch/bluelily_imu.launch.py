from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for BlueLily'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        # CHECKLIST ITEM 8: Frame ID must be 'imu_link' for ORB-SLAM3 and TF2 chain
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='Frame ID for IMU messages (must be imu_link for ORB-SLAM3)'
        ),
        
        # BlueLily IMU Bridge Node
        Node(
            package='bluelily_bridge',
            executable='bluelily_imu_node',
            name='bluelily_imu_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
            }]
        ),
        
        # CHECKLIST ITEM 9: TF2 static transform from imu_link to kinect2_link
        # This is REQUIRED for ORB-SLAM3 to fuse IMU data with camera
        # Values are initial estimates - Kalibr will provide precise calibration in Phase 1
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_kinect_tf',
            arguments=[
                '0.0', '0.0', '0.0',           # x y z translation (meters) - measure from robot head
                '0.0', '0.0', '0.0', '1.0',    # qx qy qz qw rotation (identity quaternion estimate)
                'imu_link',                     # parent frame
                'kinect2_link'                  # child frame
            ],
            output='screen'
        ),
    ])
