from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import os
import json

import yolo8_series.configs as yolo
import redoxi_common_py.configs.detection_relay as detRelayCfg
import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import redoxi_common_py.configs.detection_driver as detDriverCfg
import redoxi_common_py.configs.frame_relay as frameRelayCfg
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
    # video_source -> detection_driver -> tracker_driver -> frame_relay
    #                       |                   |
    #                    detector           tracker


class DetectorParams:
    fn_model = f"{workspace_root}/tmp/models/yolov8s-pose.onnx"
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
                input_port_config=yolo.InputPortConfig(
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
            input_port_config=detDriverCfg.InputPortConfig(
                action_name=input_action_name,
                buffer_capacity=InputCacheSize.Medium,
            ),
            output_port_config=detDriverCfg.OutputPortConfig(
                probe_topic_for_target_data="output/probe/target_data",
                probe_topic_for_source_data="output/probe/source_data",
            ),
            callee_request_port_config=detDriverCfg.OutputPortConfig(
                downstream_specs=[
                    detDriverCfg.DownstreamSpec(
                        name=DetectorParams.node_name,
                        action_name=callee_action_name,
                        # create_debug_pub=True,
                    ),
                ],
                # visualization_topic_for_source_data="vis/callee/source_data",
                # visualization_topic_for_target_data="callee/vis/target_data",
                probe_topic_for_target_data="callee/probe/target_data",
                probe_topic_for_source_data="callee/probe/source_data",
            ),
        ),
        runtime_config=detDriverCfg.DetectionDriverRuntimeConfig(
            enable_blocking_mode=False,
        ),
    )


class VideoSourceParams:
    # video_source -> detection_driver
    node_name = "video_source"
    fn_video = f"{workspace_root}/.bigdata/crowded_0820.coded.mp4"
    node_params = videoSrcCfg.VideoSourceFromUrlNodeConfig(
        init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
            video_url=fn_video,
            auto_replay=True,
            primary_output_spec=videoSrcCfg.OutputPortConfig(
                downstream_specs=[
                    videoSrcCfg.DownstreamSpec(
                        name=DetectionDriverParams.node_name,
                        action_name=DetectionDriverParams.input_action_name,
                        create_debug_pub=False,
                    ),
                ],
                # data_topic_for_source_data="data_msg/source_data",
                # data_topic_for_target_data="data_msg/target_data",
                probe_topic_for_target_data="probe/target_data",
            ),
        ),
        runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
            step_interval=StepIntervals.Medium,
            video_start_time=0,
            video_end_time=1000000,
            frame_enqueue_policy=videoSrcCfg.DeliveryPolicy(
                precondition=videoSrcCfg.DeliveryPrecondition.DontCare,
                drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
            ),
            frame_request_policy=videoSrcCfg.DeliveryPolicy(
                precondition=videoSrcCfg.DeliveryPrecondition.DontCare,
                drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
                retry_policy=videoSrcCfg.RetryPolicy(
                    number_of_retry=0,
                    wait_time_between_retry=1000,
                    wait_time_retry_response=1000,
                ),
            ),
            # output_image_size={"width": 1920, "height": 1080},
            output_image_size={"width": 1024, "height": -1},
        ),
    )


# Create container node
container = ComposableNodeContainer(
    name="main_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
        # Detection driver node
        ComposableNode(
            package="redoxi_common_nodes",
            plugin="redoxi_works::common_nodes::drivers::DetectionDriver",
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
        # Video source node
        ComposableNode(
            package="redoxi_video_reader",
            plugin="redoxi_works::video_readers::VideoSourceFromUrl",
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
        # Detector node
        ComposableNode(
            package="yolo8_series",
            plugin="redoxi_works::model_nodes::yolo8::Yolo8BodyPoseNode",
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
            ],
            "autostart": True,  # This will automatically configure and activate nodes
        }
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
            container,
            lifecycle_manager,
        ]
    )
