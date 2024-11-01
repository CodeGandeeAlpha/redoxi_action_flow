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
            simple_action_generator,
            # video_source_node,
            video_sink_node,
        ]
    )
