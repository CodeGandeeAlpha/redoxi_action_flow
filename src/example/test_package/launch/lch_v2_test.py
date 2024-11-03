from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
import json

logger = LaunchConfiguration("log_level")
log_level_arg = DeclareLaunchArgument(
    "log_level",
    default_value="debug",
    description="Logging level",
)

source_node_json_params = {
    "init_config": {
        "_time_unit": "us(1e-6)",
        "primary_output_spec": {
            "downstream_specs": [
                {
                    "name": "video_sink",
                    "action_name": "/video_sink/in/action",
                    "delivery_policy": {
                        "retry_policy": {
                            "number_of_retry": 5,
                            "fallback_number_of_retry": 3,
                            "wait_time_between_retry": 10000,
                            "fallback_wait_time_between_retry": 5000,
                            "wait_time_retry_response": 5000,
                            "fallback_wait_time_retry_response": 1000000,
                        },
                        "precondition": 0,
                        "drop_strategy": 0,
                    },
                    "use_debug_publish": False,
                }
            ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": 2,
        },
        "create_debug_pub": False,
        "debug_pub_queue_size": 10,
        "debug_pub_task_enqueue_name": "debug_port/task_enqueue",
        "debug_pub_task_drop_name": "debug_port/task_drop",
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": 10000000,
        "frame_interval": 0,
        "output_image_size": {"width": -1, "height": -1},
        "output_image_encoding": "bgr8",
        "publish_to_debug_topic": False,
        "fallback_primary_output_policy": {
            "retry_policy": {
                "fallback_number_of_retry": 3,
                "fallback_wait_time_between_retry": 5000,
                "fallback_wait_time_retry_response": 1000000,
            },
            "precondition": 0,
            "drop_strategy": 0,
        },
        "frame_request_policy": {
            "retry_policy": {
                "number_of_retry": 0,
                "fallback_number_of_retry": 3,
                "wait_time_between_retry": 5000,
                "fallback_wait_time_between_retry": 5000,
                "wait_time_retry_response": 5000,
                "fallback_wait_time_retry_response": 1000000,
            },
            "precondition": 2,
            "drop_strategy": 2,
        },
    },
}

relay_node_json_params = {
    "declare_params": {},
    "init_config": {
        "frame_receive_action_name": "in/action",
        "relayed_frame_topic_name": "out/image",
        "publish_queue_size": 10,
        "publish_raw_image": True,
        "use_async": False,
        "goal_buffer_size": 1,
        "debug_pub_enabled": True,
    },
}

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None

simple_action_generator = Node(
    package="test_package",
    executable="v2_simple_action",
    name="simple_action_generator",
    namespace="simple_action_generator",
    parameters=[
        {
            "param_as_json_string": json.dumps(
                source_node_json_params, separators=(",", ":")
            ),
        },
    ],
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", ["simple_action_generator:=", logger]],
)

video_source_node = Node(
    package="test_package",
    executable="test_video_reader_random",
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
    executable="test_video_sink",
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
    arguments=["--ros-args", "--log-level", ["video_sink:=", logger]],
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
        # SetEnvironmentVariable(
        #     "FASTRTPS_DEFAULT_PROFILES_FILE",
        #     "/soft/workspace/code/psf_ros2_ws/scripts/dds-profile.xml",
        # ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            simple_action_generator,
            # video_source_node,
            video_sink_node,
        ]
    )
