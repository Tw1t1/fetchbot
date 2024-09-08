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

    # Layer 0 - Obstacle Avoidance
    feel_force_node = Node(package="obstacle_avoidance", executable="feel_force", output="screen")
    runaway_node = Node(package="obstacle_avoidance", executable="runaway", output="screen")
    avoid_runaway_suppressor_node = Node(package="obstacle_avoidance", executable="avoid_runaway_suppressor", output="screen")
    locomotion_controller_node = Node(package="obstacle_avoidance", executable="locomotion_controller", output="screen")

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    
    # Add any actions
    ld.add_action(feel_force_node)
    ld.add_action(runaway_node)
    ld.add_action(avoid_runaway_suppressor_node)
    ld.add_action(locomotion_controller_node)

    return ld