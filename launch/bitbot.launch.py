from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ovinf_ros2"),
                    "urdf",
                    "hhfc",
                    "urdf",
                    "hhfc.urdf",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_file}
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ovinf_ros2"),
            "rviz",
            "view_robot.rviz",
        ]
    )

    bitbot_node = Node(
        package="ovinf_ros2",
        executable="main_app",
        name="main_app",
        output="screen",
    )

    robot_state_publisher_node = Node(
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
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
        bitbot_node,
    ]
    return LaunchDescription(nodes)
