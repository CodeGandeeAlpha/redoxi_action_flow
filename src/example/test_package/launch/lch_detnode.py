from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
import json

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


InputPortQueueSize = 10

DetectionNodeName = "detector"
DetectionInputActionName = "in/image_request"

VideoSourceNodeName = "video_source"
# fn_model_nano = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-dynbatch.onnx"
fn_model_nano = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-640.onnx"
fn_model_medium = (
    "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8m-pose-dynbatch.onnx"
)
fn_video = "/soft/workspace/code/psf_ros2_ws/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4"
# fn_video = "/soft/workspace/code/psf_ros2_ws/.bigdata/crowded_0820.coded.mp4"

DetectionRelayNodeName = "detection_relay"
DetectionRelayInputActionName = "in/detections"

det_node_params = {
    "declare_params": {},
    "init_config": {
        "model_configs": [
            {
                "model_path": fn_model_nano,
                "device_type": "cpu",
                "device_index": 0,
            },
            # {
            #     "model_path": fn_model_medium,
            #     "device_type": "cuda",
            #     "device_index": 0,
            # },
            # {
            #     "model_path": fn_model_medium,
            #     "device_type": "cuda",
            #     "device_index": 0,
            # },
        ],
        # "detection_request_config": {
        #     "input_port_config": {
        #         "buffer_capacity": -1,
        #         "action_name": "in/detection_request",
        #         "goal_result_expire_time": 1000000,
        #     }
        # },
        "image_request_config": {
            "input_port_config": {
                "buffer_capacity": InputPortQueueSize,
                "action_name": DetectionInputActionName,
                "goal_result_expire_time": 1000000,
            },
            "output_port_config": {
                "downstream_specs": [
                    {
                        "name": "",
                        "action_name": f"/{DetectionRelayNodeName}/{DetectionRelayInputActionName}",
                        "delivery_policy": {
                            "precondition": "dont_care",
                            "drop_strategy": "dont_care",
                        },
                        "create_debug_pub": False,
                    }
                ],
                "num_buffer_requests": 1,
                "preserve_request_order": True,
                "fallback_delivery_precondition": "dont_care",
            },
            "output_enqueue_policy": {
                "precondition": "dont_care",
                "drop_strategy": "dont_care",
            },
        },
        "visualization_topic": "debug/visualization",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": StepIntervals.VeryFast,
        "enable_blocking_mode": False,
        "model_output_config": {"conf_threshold": 0.1, "iou_threshold": 0.6},
        "enable_visualization": True,
    },
}

video_source_params = {
    "declare_params": {},
    "init_config": {
        "video_url": fn_video,
        "auto_replay": True,
        "primary_output_spec": {
            "downstream_specs": [
                {
                    "name": DetectionNodeName,
                    "action_name": f"/{DetectionNodeName}/{DetectionInputActionName}",
                    "delivery_policy": {
                        "retry_policy": {
                            "fallback_number_of_retry": 3,
                            "fallback_wait_time_between_retry": 5000,
                            "fallback_wait_time_retry_response": 1000000,
                        },
                        "precondition": "dont_care",
                        "drop_strategy": "no_drop",
                    },
                    "create_debug_pub": False,
                }
            ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "dont_care",
        },
        "create_debug_pub": True,
        "debug_pub_queue_size": 10,
        "debug_pub_task_enqueue_name": "debug_port/task_enqueue",
        "debug_pub_task_drop_name": "debug_port/task_drop",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "video_start_time": 0,
        "video_end_time": -1,
        "frame_interval": 0,
        "output_image_size": {"width": 1024, "height": -1},
        "output_image_encoding": "rgb8",
        "publish_to_debug_topic": True,
        "frame_enqueue_policy": {
            "retry_policy": {
                "number_of_retry": 5,
                "fallback_number_of_retry": 3,
                "wait_time_between_retry": 5000,
                "fallback_wait_time_between_retry": 5000,
                "wait_time_retry_response": 5000,
                "fallback_wait_time_retry_response": 1000000,
            },
            "precondition": "dont_care",
            "drop_strategy": "no_drop",
        },
        "_time_unit": "us(1e-6)",
        "step_interval": StepIntervals.Fast,
    },
}

detection_relay_params = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {
            "buffer_capacity": InputPortQueueSize,
            "action_name": f"{DetectionRelayInputActionName}",
            "goal_result_expire_time": 1000000,
        },
        "publish_detection_topic": "out/relayed_detection",
        "publish_visualization_topic": "out/relayed_visualization",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "enable_blocking_mode": False,
        "enable_visualization": True,
        "_time_unit": "us(1e-6)",
        "step_interval": StepIntervals.Fast,
    },
}

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

detection_node = Node(
    package="test_package",
    executable="yolo_body_pose_detection_node",
    name=DetectionNodeName,
    namespace=DetectionNodeName,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": json.dumps(det_node_params, separators=(",", ":")),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{DetectionNodeName}:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
)

video_source_node = Node(
    package="test_package",
    executable="video_from_url_node",
    name=VideoSourceNodeName,
    namespace=VideoSourceNodeName,
    output="screen",
    parameters=[
        {
            "param_as_json_string": json.dumps(
                video_source_params, separators=(",", ":")
            ),
        },
    ],
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", [f"{VideoSourceNodeName}:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
)

detection_relay_node = Node(
    package="test_package",
    executable="detection_relay_node",
    name=DetectionRelayNodeName,
    namespace=DetectionRelayNodeName,
    output="screen",
    parameters=[
        {
            "param_as_json_string": json.dumps(
                detection_relay_params, separators=(",", ":")
            ),
        },
    ],
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", [f"{DetectionRelayNodeName}:=", logger]]
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
            video_source_node,
            detection_node,
            detection_relay_node,
        ]
    )
