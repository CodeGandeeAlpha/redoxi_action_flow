from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
import json

import yolo8_series.configs as yolo
import redoxi_common_py.configs.detection_relay_configs as detRelayCfg

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
    Medium = 20000
    Fast = 5000
    VeryFast = 1000


# fn_model_nano = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-dynbatch.onnx"
fn_model = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8s.onnx"

det_node_name = "detector"
det_node_params = yolo.Yolo8ModelConfig(
    init_config=yolo.InitConfig(
        model_configs=[
            {
                "model_path": fn_model,
                "device_type": "cuda",
                "device_index": 0,
            },
        ],
    ),
)

det_relay_node_name = "detection_relay"
det_relay_node_params = detRelayCfg.DetectionRelayNodeConfig(
    init_config=detRelayCfg.InitConfig(
        input_port_config=None,
        publish_detection_topic="out/relayed_detection",
        publish_visualization_topic="out/relayed_visualization",
    ),
    runtime_config=detRelayCfg.RuntimeConfig(
        enable_blocking_mode=False,
        enable_visualization=True,
    ),
)

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

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
            # detection_node,
            detection_relay_node,
        ]
    )
