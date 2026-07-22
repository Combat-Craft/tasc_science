from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("science_description")

    urdf_file = PathJoinSubstitution([
        description_share,
        "urdf",
        "science_2026.urdf.xacro",
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            urdf_file,
        ]),
        value_type=str,
    )

    # Runs locally on the VM and creates TF from joint states received
    # from the Jetson.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="science",
        parameters=[
            {
                "robot_description": robot_description,
            },
        ],
        remappings=[
            ("joint_states", "/science/joint_states"),
        ],
        output="screen",
    )

    # Runs locally on the VM and publishes commands to the Jetson.
    keyboard_teleop_node = Node(
        package="science_control",
        executable="keyboard_teleop",
        output="screen",
    )

    # Runs locally so no graphical interface is needed on the Jetson.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="science",
        remappings=[
            ("/joint_states", "/science/joint_states"),
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        keyboard_teleop_node,
        rviz_node,
    ])
