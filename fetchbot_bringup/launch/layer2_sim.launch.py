from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    params_file_dec = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('ball_follow'),'config','ball_follow_params_example.yaml'),
        description='Full path to params file for all ball_follow nodes.')

    detect_only = LaunchConfiguration('detect_only')
    detect_only_dec = DeclareLaunchArgument(
        'detect_only',
        default_value='false',
        description='Doesn\'t run the follow component. Useful for just testing the detections.')
    
    follow_only = LaunchConfiguration('follow_only')
    follow_only_dec = DeclareLaunchArgument(
        'follow_only',
        default_value='false',
        description='Doesn\'t run the detect component. Useful for testing just the following. (e.g. with manually published detections)')
    
    tune_detection = LaunchConfiguration('tune_detection')
    tune_detection_dec = DeclareLaunchArgument(
        'tune_detection',
        default_value='false',
        description='Enables tuning mode for the detection')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Enables sim time for the follow node.')
    
    image_topic = LaunchConfiguration('image_topic')
    image_topic_dec = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera_sensor/image_raw',
        description='The name of the input image topic.')

    detect_node = Node(
            package='ball_follow',
            executable='detect_ball',
            parameters=[params_file, {'tuning_mode': tune_detection}],
            remappings=[('/image_in',image_topic)],
            condition=UnlessCondition(follow_only)
         )

    follow_node = Node(
            package='ball_follow',
            executable='follow_ball',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/detected_ball','/orient_home_detect_ball_inhibitor')],
            condition=UnlessCondition(detect_only),
         )

    inhibit_detect_ball_node = Node(
            package='ball_follow',
            executable='orient_home_ball_detection_inhibitor',
         )
    
    inhibit_follow_ball_node = Node(
            package='ball_follow',
            executable='grab_ball_follow_ball_inhibitor',
         )
    
    # layer 0-1 
    enable_wander = LaunchConfiguration('enable_wander')
    enable_wander_dec = DeclareLaunchArgument(
        'enable_wander',
        default_value='true',
        description='Enables the wander node')

    # Define path variables
    urdf_path = PathJoinSubstitution([FindPackageShare("fetchbot_description"), "urdf", "fetchbot.urdf.xacro"])
    rviz_config_path = PathJoinSubstitution([FindPackageShare("fetchbot_bringup"), "rviz", "urdf_config.rviz"])
    gazebo_world_path = PathJoinSubstitution([FindPackageShare("fetchbot_bringup"), "worlds", "room.world"])
    gazebo_params_file = PathJoinSubstitution([FindPackageShare("fetchbot_description"), "config", "gazebo_params.yaml"])

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(['xacro ', urdf_path])}],
    )

    # Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
        launch_arguments={
            "world": gazebo_world_path,
            "extra_gazebo_args": "--ros-args --params-file " + str(gazebo_params_file)
        }.items()
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "fetchbot"],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    # Controller Manager
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    # Layer 0 - Obstacle Avoidance
    feel_force_node = Node(package="obstacle_avoidance", executable="feel_force", output="screen")
    runaway_node = Node(package="obstacle_avoidance", executable="runaway", output="screen")
    avoid_runaway_suppressor_node = Node(package="obstacle_avoidance", executable="avoid_runaway_suppressor", output="screen")
    locomotion_controller_node = Node(package="obstacle_avoidance", executable="locomotion_controller", output="screen")

    # Layer 1 - Wander
    wander_node = Node(package="wander", executable="wander", name="wander", output="screen", condition=IfCondition(enable_wander))
    avoid_node = Node(package="wander", executable="avoid", name="avoid", output="screen")
    detect_ball_wander_inhibitor_node = Node(package="wander", executable="detect_ball_wander_inhibitor", name="detect_ball_wander_inhibitor", output="screen")
    follow_ball_wander_suppressor_node = Node(package="wander", executable="follow_ball_wander_suppressor", name="follow_ball_wander_suppressor", output="screen")
    orient_home_wander_suppressor_node = Node(package="wander", executable="orient_home_wander_suppressor", name="orient_home_wander_suppressor", output="screen")

    return LaunchDescription([
        params_file_dec,
        detect_only_dec,
        follow_only_dec,
        tune_detection_dec,
        use_sim_time_dec,
        image_topic_dec,
        detect_node,
        follow_node,
        inhibit_detect_ball_node,
        inhibit_follow_ball_node,
        enable_wander_dec,
        robot_state_pub_node,
        gazebo_node,
        spawn_entity_node,
        rviz_node,
        diff_drive_spawner,
        joint_broad_spawner,
        feel_force_node,
        runaway_node,
        avoid_runaway_suppressor_node,
        locomotion_controller_node,
        wander_node,
        avoid_node,
        detect_ball_wander_inhibitor_node,
        follow_ball_wander_suppressor_node,
        orient_home_wander_suppressor_node
    ])