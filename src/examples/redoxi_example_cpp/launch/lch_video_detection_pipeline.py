# here we show how to use launch file to run the video detection pipeline

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import redoxi_common_py.configs.detection_relay as detRelayCfg
import yolo8_series.configs as yolo8Cfg

class RunConfig:
    WorkspaceRoot = "/soft/workspace/code/redoxi_action_flow"
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
# video_source -> detection_node -> detection_relay

detection_relay_config = detRelayCfg.DetectionRelayNodeConfig(
    init_config=detRelayCfg.DetectionRelayInitConfig(
        input_port_config=detRelayCfg.InputPortConfig(
            action_name="input_detections",
        ),
        publish_detection_topic="relayed_detections",
        publish_visualization_topic="relayed_visualization",
    ),
    runtime_config=detRelayCfg.DetectionRelayRuntimeConfig(
        step_interval=1000 * 1000 // 10,  # pop input every this many macroseconds
    ),
)

detection_relay_node = Node(
    package="redoxi_samples_nodes",
    executable="node_pack_samples_DetectionRelayNode",
    name="detection_relay",
    namespace="detection_relay",
    parameters=[
        {
            RunConfig.JsonParamKey: detection_relay_config.to_json(
                ignore_none=True, compact=False
            )
        }
    ],
    arguments=get_logger_arg("detection_relay"),
)

detection_node_config = yolo8Cfg.Yolo8ModelNodeConfig(
    init_config=yolo8Cfg.Yolo8ModelInitConfig(
        # the upstream should send goals to this action server
        model_configs=[
            yolo8Cfg.ModelConfig(
                model_path=f"{RunConfig.WorkspaceRoot}/tmp/models/yolov8s.onnx",
                device_type="cuda",
                device_index=0,
            )
        ],
        image_request_config=yolo8Cfg.ImageRequestConfig(
            input_port_config=yolo8Cfg.InputPortConfig(
                action_name="input_frame",
            ),
            output_port_config=yolo8Cfg.OutputPortConfig(
                downstream_specs=[
                    yolo8Cfg.DownstreamSpec(
                        name="send_to_relay",
                        action_name="/detection_relay/input_detections",
                    ),
                ],
                visualization_topic_for_target_data="vis/target_data",
            ),
        ),
    ),
    runtime_config=yolo8Cfg.Yolo8ModelRuntimeConfig(
        step_interval=1000 * 1000 // 10,  # pop input every this many macroseconds
        enable_visualization=True,
        enable_performance_probe=True,
        model_output_config=yolo8Cfg.ModelPostprocessConfig(
            conf_threshold=0.25,
            iou_threshold=0.45,
        ),
    ),
)

detection_node = Node(
    package="yolo8_series",
    executable="node_pack_detection_yolo8_ObjectDetector",
    name="detection_node",
    namespace="detection_node",
    parameters=[
        {
            RunConfig.JsonParamKey: detection_node_config.to_json(
                ignore_none=True, compact=False
            )
        }
    ],
    arguments=get_logger_arg("detection_node"),
)

video_source_node_config = videoSrcCfg.VideoSourceFromUrlNodeConfig(
    init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
        video_url=f"{RunConfig.WorkspaceRoot}/src/extern/RedoxiTrack/data/videos/dancetrack-0039.mp4",
        # optional published topics for monitoring
        primary_output_spec=videoSrcCfg.OutputPortConfig(
            downstream_specs=[
                videoSrcCfg.DownstreamSpec(
                    name="send_to_detection",  # can be anything
                    action_name="/detection_node/input_frame",  # absolute action name of downstream input port
                )
            ],
            data_topic_for_source_data=f"data_msg/source_data",  # get reliably published data for visualization
            visualization_topic_for_target_data=f"vis/target_data",  # get lossy published data for visualization
            probe_topic_for_target_data=f"probe/target_data",  # get reliable published string for probing
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=1000 * 1000 // 1,  # time in macroseconds (1e-6 s)
        # optional, resize the frame to this size
        output_image_size={"width": 640, "height": -1},
        # retry policy, specify how to handle transmission failures
        # if not set, by default it will retry as many times as possible until success.
        # the frame data will be first enqueued to the reader node's sending buffer, and then send to downstream
        # a frame may be dropped in the following cases:
        # 1. in enqueue stage, if sending buffer is full because sending is too slow
        # 2. in delivery stage, if downstream is not responsive or rejects the frame
        # so, we have 2 retry policies, one for enqueue stage, one for delivery stage
        frame_enqueue_policy=videoSrcCfg.DeliveryPolicy(
            drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
            retry_policy=videoSrcCfg.RetryPolicy(
                number_of_retry=3,
                wait_time_between_retry=1000 * 10,  # in 1e-6 s
            ),
        ),
        frame_request_policy=videoSrcCfg.DeliveryPolicy(
            drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
            retry_policy=videoSrcCfg.RetryPolicy(
                number_of_retry=3,
                wait_time_between_retry=1000 * 10,  # in 1e-6 s
                wait_time_retry_response=1000 * 1000,  # in 1e-6 s
            ),
        ),
    ),
)

# executable name can be found in the CMakeLists.txt file of the target package
video_source_node = Node(
    package="redoxi_video_reader",
    executable="node_pack_video_readers_VideoSourceFromUrl",
    name="video_source",
    namespace="video_source",
    parameters=[
        {
            RunConfig.JsonParamKey: video_source_node_config.to_json(
                ignore_none=True, compact=False
            )
        }
    ],
    arguments=get_logger_arg("video_source"),
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
                "/video_source/video_source",
                "/detection_node/detection_node",
                "/detection_relay/detection_relay",
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
            detection_node,
            detection_relay_node,
        ]
    )
