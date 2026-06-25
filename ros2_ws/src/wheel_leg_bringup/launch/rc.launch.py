from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    estop_channel = LaunchConfiguration("estop_channel")
    estop_active_below = LaunchConfiguration("estop_active_below")

    return LaunchDescription(
        [
            DeclareLaunchArgument("estop_channel", default_value="7"),
            DeclareLaunchArgument("estop_active_below", default_value="true"),
            Node(
                package="wheel_leg_rc",
                executable="rc_ibus_node",
                name="rc_ibus_node",
                output="screen",
                parameters=[
                    {
                        "estop.channel": estop_channel,
                        "estop.active_below": estop_active_below,
                    }
                ],
            ),
        ]
    )
