from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
import json

log_level_arg = DeclareLaunchArgument(
    "log_level",
    default_value=["info"],
    description="Logging level",
)

source_node_json_params = {
    "declare_params": {
        "custom_var_1": 100.0,
        "custom_var_2": 10.0,
    },
    "runtime_config": {
        "frame_interval_ms": 10000.0,
        "step_interval_ms": 1,
        "publish_to_debug_topic": True,
    },
    "init_config": {
        "downstreams": {
            "actions": [
                {
                    "name": "/video_sink/in/action",
                    "retry_strategy": {
                        "max_retries": 5,
                        "retry_interval_ms": 50.0,
                    },
                }
            ]
        },
    },
}

relay_node_json_params = {
    "declare_params": {},
    "init_config": {
        "frame_receive_action_name": "in/action",
        "image_topic_name": "out/image",
        "compressed_image_topic_name": "out/compressed_image",
        "publish_queue_size": 10,
        "publish_raw_image": True,
        "publish_compressed_image": False,
        "use_async": True,
        "goal_buffer_size": 1,
    },
}

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None

simple_action_generator = Node(
    package="test_package",
    executable="test_simple_action_generator",
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
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            simple_action_generator,
            video_sink_node,
        ]
    )
