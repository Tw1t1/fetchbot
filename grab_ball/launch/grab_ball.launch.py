from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    params_file = LaunchConfiguration('params_file')
    params_file_dec = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('grab_ball'),'config','grab_ball_params_robot.yaml'),
        description='Full path to params file for all grab_ball nodes.')



    model_file = LaunchConfiguration('model_file')
    model_file_dec = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(get_package_share_directory('grab_ball'),'distance_models','rf_dist_model_0.3m_calib_graspel.joblib'),
        description='Full path to distance model file for grab_ball node.')
    
    
    claw_controller_node = Node(
            package='grab_ball',
            executable='claw_controller',
            parameters=[params_file],
         )
    
    grab_ball_node = Node(
            package='grab_ball',
            executable='grab_ball',
            parameters=[params_file],
         )

    return LaunchDescription([
        params_file_dec,
        model_file_dec,
        claw_controller_node,
        grab_ball_node
    ])