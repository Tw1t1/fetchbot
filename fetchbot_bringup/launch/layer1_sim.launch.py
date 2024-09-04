from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

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

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    
    # Add any actions
    ld.add_action(enable_wander_dec)
    ld.add_action(robot_state_pub_node)
    ld.add_action(gazebo_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(rviz_node)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)
    ld.add_action(feel_force_node)
    ld.add_action(runaway_node)
    ld.add_action(avoid_runaway_suppressor_node)
    ld.add_action(locomotion_controller_node)
    ld.add_action(wander_node)
    ld.add_action(avoid_node)
    ld.add_action(detect_ball_wander_inhibitor_node)
    ld.add_action(follow_ball_wander_suppressor_node)
    ld.add_action(orient_home_wander_suppressor_node)

    return ld