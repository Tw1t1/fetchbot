import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='camera_sensor',
        parameters=[{
            'image_size': [640, 480],
            'time_per_frame': [1, 6],
            'camera_frame_id': 'camera_link_optical',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_node
    ])
