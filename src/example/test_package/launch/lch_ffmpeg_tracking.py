# test lifecycle node activate and deactivate using nav2_lifecycle_manager
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import os
import json

import yolo8_series.configs as yolo
from redoxi_common_py.configs.video_reader_base import (
    InputPortConfig,
    OutputPortConfig,
    DownstreamSpec,
    DeliveryPolicy,
    RetryPolicy,
    DeliveryPrecondition,
    DropStrategy,
)
import redoxi_common_py.configs.detection_relay as detRelayCfg
import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import redoxi_common_py.configs.detection_driver as detDriverCfg
import redoxi_common_py.configs.frame_relay as frameRelayCfg
import redoxi_common_py.configs.ffmpeg_video_reader as ffmpegVideoReaderCfg
import universal_mot_trackers.node_configs as motTrackersCfg
import universal_mot_trackers.driver_configs as motTrackersDriverCfg

# Get workspace root from COLCON_PREFIX_PATH
import os

# workspace_root = os.getenv("COLCON_PREFIX_PATH", "")
# if workspace_root:
#     # Remove /install from the end if present
#     if workspace_root.endswith("/install"):
#         workspace_root = workspace_root[:-8]  # Remove last 8 characters ('/install')

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


# the graph structure is:
# video_source -> detection_driver -> tracker_driver -> null
#                       |                   |
#                    detector           tracker


class TrackerParams:
    node_name = "tracker"
    input_action_name = f"/{node_name}/in/track_request"

    node_params = motTrackersCfg.UniversalMotTrackersNodeConfig(
        init_config=motTrackersCfg.UniversalMotTrackersInitConfig(
            input_port_config=InputPortConfig(
                action_name=input_action_name,
                buffer_capacity=InputCacheSize.Medium,
            ),
        ),
    )


class TrackerDriverParams:
    node_name = "tracker_driver"
    input_action_name = f"/{node_name}/in/track_request"

    node_params = motTrackersDriverCfg.TrackerDriverNodeConfig(
        init_config=motTrackersDriverCfg.TrackerDriverInitConfig(
            input_port_config=InputPortConfig(
                action_name=input_action_name,
                buffer_capacity=InputCacheSize.Medium,
            ),
            # tracker_driver -> null
            output_port_config=OutputPortConfig(
                visualization_topic_for_target_data=f"/{node_name}/output/vis/target_data",
                probe_topic_for_target_data=f"/{node_name}/output/probe/target_data",
            ),
            # tracker_driver -> tracker
            callee_request_port_config=OutputPortConfig(
                downstream_specs=[
                    DownstreamSpec(
                        name=TrackerParams.node_name,
                        action_name=TrackerParams.input_action_name,
                    ),
                ],
            ),
        ),
    )


class DetectorParams:
    fn_model = f"{workspace_root}/tmp/models/yolov8s-pose.onnx"
    # fn_model = f"{workspace_root}/tmp/models/yolov8n-pose-dynbatch.onnx"
    node_name = "detector"
    input_action_name = f"/{node_name}/in/detection_request"
    node_params = yolo.Yolo8ModelNodeConfig(
        init_config=yolo.Yolo8ModelInitConfig(
            model_configs=[
                {
                    "model_path": fn_model,
                    "device_type": "cuda",
                    "device_index": 0,
                },
            ],
            detection_request_config=yolo.DetectionRequestConfig(
                input_port_config=InputPortConfig(
                    action_name=input_action_name,
                    buffer_capacity=InputCacheSize.Medium,
                ),
            ),
        ),
        runtime_config=yolo.Yolo8ModelRuntimeConfig(
            model_output_config=yolo.ModelPostprocessConfig(
                conf_threshold=0.2,
                iou_threshold=0.6,
                selected_class_ids=[0],  # 0=person (ultralytics convention)
            )
        ),
    )


class DetectionDriverParams:
    node_name = "detection_driver"
    input_action_name = f"/{node_name}/in/frame"
    callee_action_name = DetectorParams.input_action_name

    node_params = detDriverCfg.DetectionDriverNodeConfig(
        init_config=detDriverCfg.DetectionDriverInitConfig(
            input_port_config=InputPortConfig(
                action_name=input_action_name,
                buffer_capacity=InputCacheSize.Medium,
            ),
            # detection_driver -> tracker_driver
            output_port_config=OutputPortConfig(
                downstream_specs=[
                    DownstreamSpec(
                        name=TrackerDriverParams.node_name,
                        action_name=TrackerDriverParams.input_action_name,
                    ),
                ],
                visualization_topic_for_target_data=f"/{node_name}/output/vis/target_data",
                probe_topic_for_source_data=f"/{node_name}/output/probe/source_data",
            ),
            # detection_driver -> detector
            callee_request_port_config=OutputPortConfig(
                downstream_specs=[
                    DownstreamSpec(
                        name=DetectorParams.node_name,
                        action_name=callee_action_name,
                        # create_debug_pub=True,
                    ),
                ],
                # visualization_topic_for_source_data="vis/callee/source_data",
                visualization_topic_for_target_data=f"{node_name}/callee/vis/target_data",
                probe_topic_for_target_data=f"{node_name}/callee/probe/target_data",
                probe_topic_for_source_data=f"{node_name}/callee/probe/source_data",
            ),
        ),
    )


class VideoSourceParams:
    # video_source -> detection_driver
    node_name = "video_source"
    video_url = "/dev/video4"

    node_params = ffmpegVideoReaderCfg.FFmpegVideoReaderNodeConfig(
        init_config=ffmpegVideoReaderCfg.FFmpegVideoReaderInitConfig(
            ffmpeg_path="/usr/bin/ffmpeg",
            primary_output_spec=OutputPortConfig(
                downstream_specs=[
                    DownstreamSpec(
                        name=DetectionDriverParams.node_name,
                        action_name=DetectionDriverParams.input_action_name,
                        create_debug_pub=False,
                    ),
                ],
                # data_topic_for_source_data=f"{node_name}/data_msg/source_data",
                # data_topic_for_target_data=f"{node_name}/data_msg/target_data",
                visualization_topic_for_target_data=f"{node_name}/vis/target_data",
                probe_topic_for_target_data=f"{node_name}/probe/target_data",
            ),
        ),
        runtime_config=ffmpegVideoReaderCfg.FFmpegVideoReaderRuntimeConfig(
            ffmpeg_args=[
                "-f",
                "v4l2",
                "-input_format",
                "mjpeg",
                "-video_size",
                "1920x1080",
                "-discard",
                "nokey",
                "-thread_queue_size",
                "1",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-i",
                video_url,
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-",
            ],
            frame_width=1920,
            frame_height=1080,
            frame_channels=3,
            frame_encoding="bgr8",
            step_interval=StepIntervals.Medium,
            output_image_size={"width": 1024, "height": -1},
            output_image_encoding="rgb8",
        ),
    )


# Create container node
container = ComposableNodeContainer(
    name="main_container",
    namespace="root",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
        # Video source node
        ComposableNode(
            package="redoxi_video_reader",
            plugin="redoxi_works::node_pack::video_readers::FFmpegVideoReader",
            name=VideoSourceParams.node_name,
            parameters=[
                {
                    JsonParamKey: VideoSourceParams.node_params.to_json(
                        ignore_none=True, compact=False
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        # Detection driver node
        ComposableNode(
            package="redoxi_common_nodes",
            plugin="redoxi_works::node_pack::detection::DetectionDriver",
            name=DetectionDriverParams.node_name,
            parameters=[
                {
                    JsonParamKey: DetectionDriverParams.node_params.to_json(
                        ignore_none=True, compact=False
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        # Detector node
        ComposableNode(
            package="yolo8_series",
            plugin="redoxi_works::node_pack::detection::yolo8::BodyPoseDetector",
            name=DetectorParams.node_name,
            parameters=[
                {
                    JsonParamKey: DetectorParams.node_params.to_json(
                        ignore_none=True, compact=False
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        # Tracker driver node
        ComposableNode(
            package="universal_mot_trackers",
            plugin="redoxi_works::node_pack::tracking::MotTrackerDriver",
            name=TrackerDriverParams.node_name,
            parameters=[
                {
                    JsonParamKey: TrackerDriverParams.node_params.to_json(
                        ignore_none=True, compact=False
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        # Tracker node
        ComposableNode(
            package="universal_mot_trackers",
            plugin="redoxi_works::node_pack::tracking::UniversalMotTracker",
            name=TrackerParams.node_name,
            parameters=[
                {
                    JsonParamKey: TrackerParams.node_params.to_json(
                        ignore_none=True, compact=False
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ],
    output="screen",
    arguments=["--ros-args", "--log-level", logger],
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
                f"{DetectorParams.node_name}",
                f"{DetectionDriverParams.node_name}",
                f"{VideoSourceParams.node_name}",
                f"{TrackerParams.node_name}",
                f"{TrackerDriverParams.node_name}",
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
            container,
        ]
    )
