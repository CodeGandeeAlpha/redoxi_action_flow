from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
import json

import yolo8_series.configs as yolo
import redoxi_common_py.configs.detection_relay as detRelayCfg
import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import redoxi_common_py.configs.detection_driver as detDriverCfg

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


class StepIntervals:
    VerySlow = 3000000
    Slow = 200000
    Medium = 1000000 / 25
    Fast = 5000
    VeryFast = 1000


# video source is upstream of detection driver
video_src_node_name = "video_source"
fn_video = r"/soft/workspace/code/psf_ros2_ws/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4"
video_src_node_params = videoSrcCfg.VideoSourceFromUrlNodeConfig(
    init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
        video_url=fn_video,
        auto_replay=True,
        primary_output_spec=videoSrcCfg.OutputPortConfig(
            downstream_specs=[
                videoSrcCfg.DownstreamSpec(
                    name="detection_driver",
                    action_name="/detection_driver/in/frame",
                ),
            ],
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=StepIntervals.Medium,
    ),
)

# detection driver is downstream of video source, upstream of detector, output to detection relay
det_driver_node_name = "detection_driver"
det_driver_node_params = detDriverCfg.DetectionDriverNodeConfig(
    init_config=detDriverCfg.DetectionDriverInitConfig(
        input_port_config=detDriverCfg.InputPortConfig(
            action_name="in/frame",
        ),
        output_port_config=detDriverCfg.OutputPortConfig(
            downstream_specs=[
                detDriverCfg.DownstreamSpec(
                    name="detection_relay",
                    action_name="/detection_relay/in/detections",
                ),
            ],
        ),
        callee_request_port_config=detDriverCfg.OutputPortConfig(
            downstream_specs=[
                detDriverCfg.DownstreamSpec(
                    name="detector",
                    action_name="/detector/in/detection_request",
                ),
            ],
        ),
    ),
    runtime_config=detDriverCfg.DetectionDriverRuntimeConfig(
        callee_request_enqueue_policy=None,
        driver_output_enqueue_policy=None,
        enable_blocking_mode=False,
    ),
)

# fn_model_nano = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-dynbatch.onnx"
fn_model = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8s.onnx"
det_node_name = "detector"
det_node_params = yolo.Yolo8ModelNodeConfig(
    init_config=yolo.Yolo8ModelInitConfig(
        model_configs=[
            yolo.ModelConfig(
                model_path=fn_model,
                device_type="cuda",
                device_index=0,
            ),
        ],
        detection_request_config=yolo.DetectionRequestConfig(
            input_port_config=yolo.InputPortConfig(
                action_name="in/detection_request",
            ),
        ),
    ),
    runtime_config=yolo.Yolo8ModelRuntimeConfig(
        model_output_config=yolo.ModelPostprocessConfig(
            conf_threshold=0.25,
            iou_threshold=0.5,
        ),
        enable_visualization=True,
        step_interval=StepIntervals.Fast,
    ),
)

det_relay_node_name = "detection_relay"
det_relay_node_params = detRelayCfg.DetectionRelayNodeConfig(
    init_config=detRelayCfg.DetectionRelayInitConfig(
        input_port_config=detRelayCfg.InputPortConfig(
            action_name="in/detections",
        ),
        publish_detection_topic="out/relayed_detection",
        publish_visualization_topic="out/relayed_visualization",
    ),
    runtime_config=detRelayCfg.DetectionRelayRuntimeConfig(
        enable_blocking_mode=False,
        enable_visualization=True,
    ),
)


# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

det_driver_node = Node(
    package="test_package",
    executable="detection_driver_node",
    name=det_driver_node_name,
    namespace=det_driver_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": det_driver_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

video_src_node = Node(
    package="test_package",
    executable="video_from_url_node",
    name=video_src_node_name,
    namespace=video_src_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": video_src_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

detection_node = Node(
    package="test_package",
    executable="yolo_object_detection_node",
    name=det_node_name,
    namespace=det_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": det_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{det_node_name}:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
)

detection_relay_node = Node(
    package="test_package",
    executable="detection_relay_node",
    name=det_relay_node_name,
    namespace=det_relay_node_name,
    output="screen",
    parameters=[
        {
            "param_as_json_string": det_relay_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", [f"{det_relay_node_name}:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
)


def generate_launch_description():
    # Set environment variables for ROS 2 logging format
    env_var_settings = [
        SetEnvironmentVariable(
            "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}][{time}]: {message}"
        ),
        SetEnvironmentVariable(
            "ROS_LOG_DIR", "/soft/workspace/code/psf_ros2_ws/tmp/roslog"
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            detection_node,
            detection_relay_node,
            video_src_node,
            det_driver_node,
        ]
    )
