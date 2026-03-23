import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_behavior")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    start_referee_bridge = LaunchConfiguration("start_referee_bridge")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    pty_path = LaunchConfiguration("pty_path")

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "sentry_behavior_reality.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )
    declare_start_referee_bridge_cmd = DeclareLaunchArgument(
        "start_referee_bridge",
        default_value="false",
        description="Start the serial proxy referee bridge",
    )
    declare_serial_port_cmd = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Physical referee serial port",
    )
    declare_serial_baud_cmd = DeclareLaunchArgument(
        "serial_baud",
        default_value="115200",
        description="Physical referee serial baud rate",
    )
    declare_pty_path_cmd = DeclareLaunchArgument(
        "pty_path",
        default_value="/tmp/referee_pty",
        description="Stable PTY path consumed by dji_referee_protocol",
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package="pb2025_sentry_behavior",
                executable="reality_referee_bridge.py",
                name="reality_referee_bridge",
                output="screen",
                condition=IfCondition(start_referee_bridge),
                parameters=[
                    {
                        "serial_port_normal": serial_port,
                        "serial_baud_normal": serial_baud,
                        "pty_path": pty_path,
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="pb2025_sentry_behavior_server",
                name="pb2025_sentry_behavior_server",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="pb2025_sentry_behavior_client",
                name="pb2025_sentry_behavior_client",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_start_referee_bridge_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_serial_baud_cmd)
    ld.add_action(declare_pty_path_cmd)
    ld.add_action(bringup_cmd_group)
    return ld
