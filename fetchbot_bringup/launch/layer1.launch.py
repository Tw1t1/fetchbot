from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("fetchbot_description"), "urdf", "fetchbot.urdf.xacro"]
            ),        
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("fetchbot_description"),
            "config",
            "diffdrive_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("fetchbot_bringup"), "rviz", "urdf_config.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_cont/cmd_vel", "/cmd_vel"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    )

    sllidar_a1 = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type':"serial",
                        'serial_port': "/dev/ttyUSB0", 
                        'serial_baudrate': 115200, 
                        'frame_id': "laser",
                        'inverted': False, 
                        'angle_compensate': True}],
        output='screen'
    )

    # layer 0 - obstacle avoidance
    feel_force = Node(
        package='obstacle_avoidance',
        executable='feel_force',
        name='feel_force',
        output='screen'
    )

    runaway = Node(
        package='obstacle_avoidance',
        executable='runaway',
        name='runaway',
        output='screen'
    )

    avoid_runaway_suppressor = Node(
        package='obstacle_avoidance',
        executable='avoid_runaway_suppressor',
        name='avoid_runaway_suppressor',
        output='screen'
    )

    indicator = Node(
        package='obstacle_avoidance',
        executable='indicator',
        name='indicator',
        output='screen'
    )

    locomotion_controller = Node(
        package='obstacle_avoidance',
        executable='locomotion_controller',
        name='locomotion_controller',
        output='screen'
    )

    bumper = Node(
        package='obstacle_avoidance',
        executable='bumper',
        name='bumper',
        output='screen'
    )

    # layer 1 - wander
    wander = Node(
        package='wander',
        executable='wander',
        name='wander',
        output='screen',
        remappings=[("/wander", "/wander_ignore")]
    )

    avoid = Node(
        package='wander',
        executable='avoid',
        name='avoid',
        output='screen'
    )

    detect_ball_wander_inhibitor = Node(
        package='wander',
        executable='detect_ball_wander_inhibitor',
        name='detect_ball_wander_inhibitor',
        output='screen'
    )

    follow_ball_wander_suppressor = Node(
        package='wander',
        executable='follow_ball_wander_suppressor',
        name='follow_ball_wander_suppressor',
        output='screen'
    )

    orient_home_wander_suppressor = Node(
        package='wander',
        executable='orient_home_wander_suppressor',
        name='orient_home_wander_suppressor',
        output='screen'
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        sllidar_a1,
        feel_force,
        runaway,
        avoid_runaway_suppressor,
        indicator,
        locomotion_controller,
        bumper,
        wander,
        avoid,
        detect_ball_wander_inhibitor,
        follow_ball_wander_suppressor,
        orient_home_wander_suppressor,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)