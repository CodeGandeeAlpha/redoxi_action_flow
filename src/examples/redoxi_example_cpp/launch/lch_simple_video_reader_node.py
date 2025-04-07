# launch simple video reader node
# make sure you have PYTHONPATH set currectly, you can use the following command to set it:
# env | grep PYTHONPATH
# and then copy the PYTHONPATH to .env file

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import redoxi_common_py.configs.video_source_from_url as videoSrcCfg


class RunConfig:
    WorkspaceRoot = "/soft/workspace/code/psf_ros2_ws"
    LogLevel = "info"
    JsonParamKey = "param_as_json_string"  # used by ros nodes


logger = LaunchConfiguration("log_level")
log_level_arg = DeclareLaunchArgument(
    "log_level",
    default_value=RunConfig.LogLevel,
    description="Logging level",
)


def get_logger_arg(node_name: str):
    return ["--ros-args", "--log-level", [f"{node_name}:=", logger]]


# the pipeline structure is:
# video_source -> null

video_source_node_config = videoSrcCfg.VideoSourceFromUrlNodeConfig(
    init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
        video_url=f"{RunConfig.WorkspaceRoot}/src/extern/RedoxiTrack/data/videos/dancetrack-0039.mp4",
        # optional published topics for monitoring
        primary_output_spec=videoSrcCfg.OutputPortConfig(
            data_topic_for_source_data=f"data_msg/source_data",  # get reliably published data for visualization
            visualization_topic_for_target_data=f"vis/target_data",  # get lossy published data for visualization
            probe_topic_for_target_data=f"probe/target_data",  # get reliable published string for probing
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=1000 // 25 * 1000,  # time in macroseconds (1e-6 s)
        # optional, resize the frame to this size
        output_image_size={"width": 1024, "height": -1},
    ),
)

# executable name can be found in the CMakeLists.txt file of the target package
video_source_node = Node(
    package="redoxi_video_reader",
    executable="node_pack_video_readers_VideoSourceFromUrl",
    name="my_video_source",
    namespace="my_namespace",
    parameters=[
        {
            RunConfig.JsonParamKey: video_source_node_config.to_json(
                ignore_none=True, compact=False
            )
        }
    ],
    arguments=get_logger_arg("my_video_source"),
)


# Add lifecycle manager to handle transitions
lifecycle_manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager",
    namespace="my_namespace",
    output="screen",
    parameters=[
        {
            "node_names": [
                "my_video_source",
            ],
            "bond_timeout": 0.0,
            "autostart": True,  # This will automatically configure and activate nodes
        },
    ],
)


def generate_launch_description():
    # Set environment variables for ROS 2 logging format
    env_var_settings = [
        SetEnvironmentVariable(
            "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}][{time}]: {message}"
        ),
        SetEnvironmentVariable("ROS_LOG_DIR", f"{RunConfig.WorkspaceRoot}/tmp/roslog"),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            lifecycle_manager,
            video_source_node,
        ]
    )
