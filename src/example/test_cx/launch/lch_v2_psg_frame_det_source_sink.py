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


psg_frame_det_source_sink_node_json_params = {
    "declare_params": {},
    "init_config": {
        "primary_output_spec": {
            "downstream_specs": [
                {
                    "name": "psg_detector",
                    "action_name": "/detector_node/model_process_frame_action",
                }
            ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "any_downstream_ready",
        },
        "create_debug_pub": True,
        "_time_unit": "us(1e-6)",
        "enable_blocking_mode": False,
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": 10000000,
        "frame_interval": 0,
        "output_image_size": {"width": 640, "height": 480},
        "output_image_encoding": "bgr8",
        "publish_to_debug_topic": False,
        "frame_request_policy": {
            "precondition": "any_downstream_ready",
            "drop_strategy": "no_drop",
        },
        "frame_enqueue_policy": {
            "precondition": "any_downstream_ready",
            "drop_strategy": "no_drop",
        },
    },
}


# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []


psg_frame_det_source_sink_node = Node(
    package="test_cx",
    executable="v2_psg_frame_det_source_sink",
    name="psg_frame_det_source_sink_node",
    namespace="psg_frame_det_source_sink_node",
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_frame_det_source_sink_node_json_params, separators=(",", ":")
            ),
        },
    ],
    prefix=common_prefix,
    arguments=[
        "--ros-args",
        "--log-level",
        ["psg_frame_det_source_sink_node:=", logger],
    ]
    + common_ros_args,
)

psg_detector_node_model = Node(
    package="psg_detector",
    executable="ddq_detector_node.py",
    name="detector_node",
    namespace="psg_detector",
    prefix=common_prefix,
)


def generate_launch_description():
    # Set environment variables for ROS 2 logging format
    env_var_settings = [
        SetEnvironmentVariable(
            "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}][{time}]: {message}"
        ),
        SetEnvironmentVariable("ROS_LOG_DIR", "/3d/chengxiao/code/psf_ros2_ws/tmp/"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            psg_frame_det_source_sink_node,
            psg_detector_node_model,
        ]
    )
