from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_stm32_bridge = LaunchConfiguration("use_stm32_bridge")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_stm32_bridge", default_value="true"),
            Node(
                package="wheel_leg_stm32_bridge",
                executable="wheel_leg_stm32_bridge_node",
                name="wheel_leg_stm32_bridge",
                output="screen",
                condition=IfCondition(use_stm32_bridge),
            ),
            Node(
                package="wheel_leg_control",
                executable="wheel_leg_controller_node",
                name="wheel_leg_controller",
                output="screen",
            ),
        ]
    )
