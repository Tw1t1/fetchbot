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

    # Layer 1 - Wander
    wander_node = Node(package="wander", executable="wander", name="wander", output="screen")
    avoid_node = Node(package="wander", executable="avoid", name="avoid", output="screen")
    detect_ball_wander_inhibitor_node = Node(package="wander", executable="detect_ball_wander_inhibitor", name="detect_ball_wander_inhibitor", output="screen")
    follow_ball_wander_suppressor_node = Node(package="wander", executable="follow_ball_wander_suppressor", name="follow_ball_wander_suppressor", output="screen")
    orient_home_wander_suppressor_node = Node(package="wander", executable="orient_home_wander_suppressor", name="orient_home_wander_suppressor", output="screen")

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    
    # Add any actions
    ld.add_action(wander_node)
    ld.add_action(avoid_node)
    ld.add_action(detect_ball_wander_inhibitor_node)
    ld.add_action(follow_ball_wander_suppressor_node)
    ld.add_action(orient_home_wander_suppressor_node)

    return ld