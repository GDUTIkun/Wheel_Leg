from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_controller = LaunchConfiguration("run_controller")
    run_simulator = LaunchConfiguration("run_simulator")
    sim_bin = LaunchConfiguration("sim_bin")
    scene_file = LaunchConfiguration("scene_file")
    control_params = LaunchConfiguration("control_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_controller", default_value="true"),
            DeclareLaunchArgument("run_simulator", default_value="true"),
            DeclareLaunchArgument(
                "sim_bin",
                default_value=EnvironmentVariable(
                    "WHEEL_LEG_SIM_BIN",
                    default_value="ros2_ws/build/wheel_leg_simulate_ros2/wheel_leg_simulate",
                ),
            ),
            DeclareLaunchArgument(
                "scene_file",
                default_value=EnvironmentVariable(
                    "WHEEL_LEG_SCENE_FILE",
                    default_value="sim/mujoco/scenes/scence.xml",
                ),
            ),
            DeclareLaunchArgument(
                "control_params",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("wheel_leg_bringup"),
                        "config",
                        "control_sim.yaml",
                    ]
                ),
            ),
            Node(
                package="wheel_leg_control",
                executable="wheel_leg_controller_node",
                name="wheel_leg_controller",
                output="screen",
                condition=IfCondition(run_controller),
                parameters=[control_params],
            ),
            ExecuteProcess(
                cmd=[sim_bin, scene_file],
                output="screen",
                condition=IfCondition(run_simulator),
            ),
        ]
    )
