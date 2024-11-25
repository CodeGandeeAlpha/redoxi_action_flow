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

source_node_json_params = {
    "init_config": {
        "video_url": "/soft/workspace/code/psf_ros2_ws/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4",
        "primary_output_spec": {
            "downstream_specs": [
                {
                    "name": "video_sink",
                    "action_name": "/video_sink/in/action",
                    "delivery_policy": {
                        "retry_policy": {
                            "number_of_retry": 5,
                            "wait_time_between_retry": 10000,
                            "wait_time_retry_response": 5000,
                        },
                        "precondition": "dont_care",
                        "drop_strategy": "dont_care",
                    },
                    "create_debug_pub": True,
                }
            ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "any_downstream_ready",
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
        "output_image_size": {"width": 800, "height": -1},
        "output_image_encoding": "rgb8",
        "publish_to_debug_topic": False,
        "frame_enqueue_policy": {
            "retry_policy": {
                "number_of_retry": 5,
                "fallback_number_of_retry": 3,
                "wait_time_between_retry": 5000,
                "fallback_wait_time_between_retry": 5000,
                "wait_time_retry_response": 5000,
                "fallback_wait_time_retry_response": 1000000,
            },
            "precondition": "any_downstream_ready",
            "drop_strategy": "drop_as_needed",
        },
        "_time_unit": "us(1e-6)",
        "step_interval": 50000,
    },
}

relay_node_json_params = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {
            "buffer_capacity": 1,
            "action_name": "in/action",
            "goal_result_expire_time": 1000000,
        },
        "publish_topic": "out/relayed_frame",
        "debug_topic_frame_accepted": "debug_port/frame_accepted",
        "debug_topic_frame_rejected": "debug_port/frame_rejected",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "enable_blocking_mode": False,
        "enable_debug_topics": True,
        "_time_unit": "us(1e-6)",
        "step_interval": 5000,
    },
}

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

video_source_node = Node(
    package="test_package",
    executable="test_video_from_url",
    name="video_source",
    namespace="video_source",
    parameters=[
        {
            "param_as_json_string": json.dumps(
                source_node_json_params, separators=(",", ":")
            ),
        },
    ],
    prefix=common_prefix,
)

video_sink_node = Node(
    package="test_package",
    executable="v2_video_sink",
    name="video_sink",
    namespace="video_sink",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                relay_node_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["video_sink:=", logger]] + common_ros_args,
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
            video_sink_node,
        ]
    )
