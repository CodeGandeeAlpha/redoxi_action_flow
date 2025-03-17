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
import redoxi_common_py.configs.frame_relay as frameRelayCfg
import universal_mot_trackers.node_configs as motTrackersCfg
import universal_mot_trackers.driver_configs as motTrackersDriverCfg

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


frame_relay_node_name = "frame_relay"
frame_relay_node_params = frameRelayCfg.FrameRelayNodeConfig(
    init_config=frameRelayCfg.FrameRelayNodeInitConfig(
        input_port_config=None,
    ),
    runtime_config=frameRelayCfg.FrameRelayNodeRuntimeConfig(
        enable_blocking_mode=False,
        enable_debug_topics=True,
    ),
)

tracker_driver_node_name = "tracker_driver"
tracker_driver_node_params = motTrackersDriverCfg.TrackerDriverNodeConfig(
    init_config=motTrackersDriverCfg.TrackerDriverInitConfig(
        input_port_config=None,
        output_port_config=None,
        callee_request_port_config=None,
    ),
    runtime_config=motTrackersDriverCfg.TrackerDriverRuntimeConfig(
        enable_blocking_mode=False,
    ),
)


tracker_node_name = "tracker"
tracker_node_params = motTrackersCfg.UniversalMotTrackersNodeConfig(
    init_config=motTrackersCfg.UniversalMotTrackersInitConfig(
        input_port_config=None,
        publish_visualization_topic="out/visualization",
        publish_probe_topic="out/probe",
        preferred_image_size=None,
    ),
    runtime_config=motTrackersCfg.UniversalMotTrackersRuntimeConfig(
        enable_blocking_mode=False,
        enable_visualization=True,
        enable_performance_probe=True,
    ),
)

# fn_model_nano = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-dynbatch.onnx"
fn_model = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8s.onnx"
det_node_name = "detector"
det_node_params = yolo.Yolo8ModelNodeConfig(
    init_config=yolo.Yolo8ModelInitConfig(
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
    init_config=detRelayCfg.DetectionRelayInitConfig(
        input_port_config=None,
        publish_detection_topic="out/relayed_detection",
        publish_visualization_topic="out/relayed_visualization",
    ),
    runtime_config=detRelayCfg.DetectionRelayRuntimeConfig(
        enable_blocking_mode=False,
        enable_visualization=True,
    ),
)

video_src_node_name = "video_source"
video_src_node_params = videoSrcCfg.VideoSourceFromUrlNodeConfig(
    init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
        video_url=r"/soft/workspace/code/psf_ros2_ws/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4",
        auto_replay=True,
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=StepIntervals.VerySlow,
    ),
)

det_driver_node_name = "detection_driver"
det_driver_node_params = detDriverCfg.DetectionDriverNodeConfig(
    init_config=detDriverCfg.DetectionDriverInitConfig(
        input_port_config=None,
        output_port_config=None,
        callee_request_port_config=None,
    ),
    runtime_config=detDriverCfg.DetectionDriverRuntimeConfig(
        callee_request_enqueue_policy=None,
        driver_output_enqueue_policy=None,
        enable_blocking_mode=False,
    ),
)

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

frame_relay_node = Node(
    package="test_package",
    executable="frame_relay_node",
    name=frame_relay_node_name,
    namespace=frame_relay_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": frame_relay_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

tracker_driver_node = Node(
    package="test_package",
    executable="mot_tracker_driver",
    name=tracker_driver_node_name,
    namespace=tracker_driver_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": tracker_driver_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

tracker_node = Node(
    package="test_package",
    executable="mot_tracker_node",
    name=tracker_node_name,
    namespace=tracker_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": tracker_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

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
            # detection_node,
            # detection_relay_node,
            # video_src_node,
            # det_driver_node,
            # tracker_node,
            # tracker_driver_node,
            frame_relay_node,
        ]
    )
