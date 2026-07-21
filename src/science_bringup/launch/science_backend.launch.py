from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("science_description")
    control_share = FindPackageShare("science_control")

    urdf_file = PathJoinSubstitution([
        description_share,
        "urdf",
        "science_2026.urdf.xacro",
    ])

    controller_config = PathJoinSubstitution([
        control_share,
        "config",
        "controllers.yaml",
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            urdf_file,
        ]),
        value_type=str,
    )

    # Runs the ros2_control hardware interface on the Jetson.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="science",
        parameters=[
            {
                "robot_description": robot_description,
            },
            controller_config,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/science/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    science_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "science_velocity_controller",
            "--controller-manager",
            "/science/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    beaker_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "beaker_position_controller",
            "--controller-manager",
            "/science/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    # Allow time for the ESP32 and Phidget to attach.
    delayed_joint_state_broadcaster = TimerAction(
        period=7.0,
        actions=[
            joint_state_broadcaster_spawner,
        ],
    )

    delayed_motion_controllers = TimerAction(
        period=9.0,
        actions=[
            science_velocity_controller_spawner,
            beaker_position_controller_spawner,
        ],
    )

    return LaunchDescription([
        control_node,
        delayed_joint_state_broadcaster,
        delayed_motion_controllers,
    ])
