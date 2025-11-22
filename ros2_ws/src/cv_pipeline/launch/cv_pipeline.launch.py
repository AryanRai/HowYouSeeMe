from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/kinect2/qhd/image_color',
            description='RGB image topic'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/kinect2/qhd/image_depth',
            description='Depth image topic'
        ),
        DeclareLaunchArgument(
            'max_fps',
            default_value='5.0',
            description='Maximum processing FPS'
        ),
        
        Node(
            package='cv_pipeline',
            executable='cv_pipeline_node',
            name='cv_pipeline_node',
            output='screen',
            parameters=[{
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'max_fps': LaunchConfiguration('max_fps'),
                'python_env': '/home/aryan/anaconda3/envs/howyouseeme',
            }]
        ),
    ])
