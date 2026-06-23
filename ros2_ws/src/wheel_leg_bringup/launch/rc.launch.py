from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="wheel_leg_rc",
                executable="rc_ibus_node",
                name="rc_ibus_node",
                output="screen",
            ),
        ]
    )
