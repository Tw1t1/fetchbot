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
        description='Full path to params file for all ball_follow nodes.')


    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Enables sim time for the follow node.')
    

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_topic_dec = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/grab_ball_follow_ball_inhibitore',
        description='The name of the output command vel topic.')


    claw_controller_node = Node(
            package='grab_ball',
            executable='claw_controller',
            parameters=[params_file],
         )

    locomotion_controller_node = Node(
            package='grab_ball',
            executable='locomotion_controller',
            parameters=[params_file],
            remappings=[('/cmd_vel', cmd_vel_topic)],
         )

    grab_ball_node = Node(
            package='grab_ball',
            executable='grab_ball',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
         )


    return LaunchDescription([

        params_file_dec,
        use_sim_time_dec,
        cmd_vel_topic_dec,
        claw_controller_node,
        locomotion_controller_node,
        grab_ball_node

    ])