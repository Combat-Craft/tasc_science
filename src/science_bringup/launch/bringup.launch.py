from launch import LaunchDescription
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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
            },
            controller_config,
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
    )

    science_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "science_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
    )

    beaker_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "beaker_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        science_velocity_controller_spawner,
        beaker_position_controller_spawner,
        rviz_node,
    ])
