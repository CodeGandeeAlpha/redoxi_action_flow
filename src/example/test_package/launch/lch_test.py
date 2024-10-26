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
    "timeunit": "ms",  # default time unit for all parameters, unless otherwise specified
    "declare_params": {
        "custom_var_1": 100.0,  # custom parameter example
        "custom_var_2": 10.0,  # custom parameter example
    },
    "runtime_config": {
        "frame_interval_ms": 2,  # The frame interval in ms, 0 means as fast as possible
        "step_interval_ms": 1,  # The step interval in ms
        "publish_to_debug_topic": True,  # Whether to publish to debug topic
        "delivery_policy_fallback": {
            "number_of_enqueue_retry": 0,  # Number of times to retry enqueueing
            "wait_time_between_enqueue_retry": 10.0,  # Wait time between enqueue retries in ms
            "number_of_delivery_retry": 10,  # Number of times to retry delivery
            "wait_time_between_delivery_retry": 20.0,  # Wait time between delivery retries in ms
            "wait_time_for_delivery_response": 100.0,  # Wait time for delivery response in ms
        },
        "delivery_options": {
            "frame_payload_type": "uncompressed",  # can be "uncompressed", "uncompressed_by_shared_memory", "jpeg_encoded", "png_encoded"
            "drop_frame_strategy": "no_drop",  # can be "no_drop" or "drop_as_needed"
            "jpeg_quality": 90,  # only valid when frame_payload_type is "jpeg_encoded"
            "num_buffer_frames": 1,  # number of frames to buffer waiting for delivery
        },
    },
    "init_config": {
        "use_debug_pub": True,  # Whether to create debug publisher
        "downstreams": {
            "actions": [
                {
                    "name": "/video_sink/in/action",  # Name of the downstream action
                    "delivery_policy": {
                        "number_of_enqueue_retry": 10,  # Number of times to retry enqueueing
                        "wait_time_between_enqueue_retry": 5.0,  # Wait time between enqueue retries in ms
                        "number_of_delivery_retry": 10,  # Number of times to retry delivery
                        "wait_time_between_delivery_retry": 10.0,  # Wait time between delivery retries in ms
                        "wait_time_for_delivery_response": 50.0,  # Wait time for delivery response in ms
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
            # simple_action_generator,
            video_source_node,
            video_sink_node,
        ]
    )
