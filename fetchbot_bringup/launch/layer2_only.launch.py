from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []


    
    params_file= PathJoinSubstitution([FindPackageShare("ball_follow"), "config", "ball_follow_params_example.yaml"])

    detect_node = Node(
            package='ball_follow',
            executable='detect_ball',
            parameters=[params_file],
            remappings=[('/image_in','/camera_sensor/image_raw')],
         )

    follow_node = Node(
            package='ball_follow',
            executable='follow_ball',
            parameters=[params_file],
            remappings=[('/detected_ball','/orient_home_detect_ball_inhibitor')],
         )

    inhibit_detect_ball_node = Node(
            package='ball_follow',
            executable='orient_home_ball_detection_inhibitor',
         )
    
    inhibit_follow_ball_node = Node(
            package='ball_follow',
            executable='grab_ball_follow_ball_inhibitor',
         )

    ld = LaunchDescription(declared_arguments)
    
    # Add any actions
    ld.add_action(detect_node)
    ld.add_action(follow_node)
    ld.add_action(inhibit_detect_ball_node)
    ld.add_action(inhibit_follow_ball_node)
    

    return ld