from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import json

json_params = {
    "declare_params": {
        "custom_var_1": 100.0,
        "custom_var_2": 10.0,
    },
    "runtime_config": {
        "frame_interval_ms": 10000.0,
        "step_interval_ms": 1.0,
    },
    "init_config": {
        "downstreams": {
            "actions": {
                "/video_sink/in/action": {
                    "retry_strategy": {
                        "max_retries": 3,
                        "retry_interval_ms": 50.0,
                    }
                }
            },
        },
    },
}

simple_action_generator = Node(
    package="test_package",
    executable="test_simple_action_generator",
    name="simple_action_generator",
    namespace="simple_action_generator",
    parameters=[
        {
            "param_as_json_string": json.dumps(json_params, separators=(",", ":")),
        },
    ],
)

video_source_node = Node(
    package="test_package",
    executable="test_video_reader_random",
    name="video_source",
    namespace="video_source",
    parameters=[
        {
            "param_as_json_string": json.dumps(json_params, separators=(",", ":")),
        },
    ],
)

video_sink_node = Node(
    package="test_package",
    executable="test_video_sink",
    name="video_sink",
    namespace="video_sink",
)


def generate_launch_description():
    # Set environment variables for ROS 2 logging format
    env_var_settings = [
        SetEnvironmentVariable(
            "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}][{time}]: {message}"
        ),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            simple_action_generator,
            video_sink_node,
        ]
    )
