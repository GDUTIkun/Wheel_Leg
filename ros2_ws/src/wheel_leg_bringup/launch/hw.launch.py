from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_stm32_bridge = LaunchConfiguration("use_stm32_bridge")
    serial_device = LaunchConfiguration("serial_device")
    baud_rate = LaunchConfiguration("baud_rate")
    state_timeout_sec = LaunchConfiguration("state_timeout_sec")
    command_timeout_sec = LaunchConfiguration("command_timeout_sec")
    command_enable = LaunchConfiguration("command_enable")
    publish_imu = LaunchConfiguration("publish_imu")
    publish_joint_states = LaunchConfiguration("publish_joint_states")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_stm32_bridge", default_value="true"),
            DeclareLaunchArgument("serial_device", default_value="/dev/ttyAMA4"),
            DeclareLaunchArgument("baud_rate", default_value="921600"),
            DeclareLaunchArgument("state_timeout_sec", default_value="0.1"),
            DeclareLaunchArgument("command_timeout_sec", default_value="0.1"),
            DeclareLaunchArgument("command_enable", default_value="false"),
            DeclareLaunchArgument("publish_imu", default_value="true"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            Node(
                package="wheel_leg_stm32_bridge",
                executable="wheel_leg_stm32_bridge_node",
                name="wheel_leg_stm32_bridge",
                output="screen",
                condition=IfCondition(use_stm32_bridge),
                parameters=[
                    {
                        "serial_device": serial_device,
                        "baud_rate": baud_rate,
                        "state_timeout_sec": state_timeout_sec,
                        "command_timeout_sec": command_timeout_sec,
                        "command_enable": command_enable,
                        "publish_imu": publish_imu,
                        "publish_joint_states": publish_joint_states,
                    }
                ],
            ),
            Node(
                package="wheel_leg_control",
                executable="wheel_leg_controller_node",
                name="wheel_leg_controller",
                output="screen",
            ),
        ]
    )
