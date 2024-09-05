from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Define path variables
    urdf_path = PathJoinSubstitution([FindPackageShare("fetchbot_description"), "urdf", "fetchbot.urdf.xacro"])
    rviz_config_path = PathJoinSubstitution([FindPackageShare("fetchbot_bringup"), "rviz", "urdf_config.rviz"])
    gazebo_world_path = PathJoinSubstitution([FindPackageShare("fetchbot_bringup"), "worlds", "room.world"])
    gazebo_params_file = PathJoinSubstitution([FindPackageShare("fetchbot_description"), "config", "gazebo_params.yaml"])
    obstacle_avoidance_params_file = PathJoinSubstitution([FindPackageShare("obstacle_avoidance"), "config", "obstacle_avoidance_params.yaml"])

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

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    
    # Add any actions
    ld.add_action(robot_state_pub_node)
    ld.add_action(gazebo_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(rviz_node)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    return ld