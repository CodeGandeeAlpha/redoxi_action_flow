# test lifecycle node activate and deactivate using nav2_lifecycle_manager
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import os
import json

import redoxi_common_py.configs.ffmpeg_video_reader as ffmpegVideoReaderCfg

workspace_root = "/soft/workspace/code/psf_ros2_ws"

try:
    from attrs import asdict
except ImportError:
    from attr import asdict

logger = LaunchConfiguration("log_level")
log_level_arg = DeclareLaunchArgument(
    "log_level",
    default_value="info",
    description="Logging level",
)


def get_logger_arg(node_name: str):
    return ["--ros-args", "--log-level", [f"{node_name}:=", logger]]


JsonParamKey = "param_as_json_string"


class StepIntervals:
    VerySlow = 3000000
    Slow = 200000
    Medium = 1000000 / 25
    Fast = 5000
    VeryFast = 1000


class InputCacheSize:
    Small = 1
    Medium = 2
    Large = 4


class VideoSourceParams:
    # video_source -> detection_driver
    node_name = "video_source"
    video_url = "udp://localhost:5555/live"
    node_params = ffmpegVideoReaderCfg.FFmpegVideoReaderNodeConfig(
        init_config=ffmpegVideoReaderCfg.FFmpegVideoReaderInitConfig(
            ffmpeg_path="/usr/bin/ffmpeg",
            ffmpeg_args=[
                "-i",
                video_url,
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "pipe:",
            ],
            frame_width=1920,
            frame_height=1080,
            frame_channels=3,
            frame_encoding="bgr8",
            primary_output_spec=ffmpegVideoReaderCfg.OutputPortConfig(
                visualization_topic_for_target_data=f"vis/target_data",
            ),
        ),
        runtime_config=ffmpegVideoReaderCfg.FFmpegVideoReaderRuntimeConfig(
            step_interval=StepIntervals.Medium,
        ),
    )


video_reader_node = Node(
    package="redoxi_video_reader",
    executable="node_pack_video_readers_FFmpegVideoReader",
    name=VideoSourceParams.node_name,
    parameters=[
        {
            JsonParamKey: VideoSourceParams.node_params.to_json(
                ignore_none=True, compact=False
            )
        },
    ],
)

# Add lifecycle manager to handle transitions
lifecycle_manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager",
    output="screen",
    parameters=[
        {
            "node_names": [
                f"{VideoSourceParams.node_name}",
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
        SetEnvironmentVariable("ROS_LOG_DIR", f"{workspace_root}/tmp/roslog"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            lifecycle_manager,
            video_reader_node,
        ]
    )
