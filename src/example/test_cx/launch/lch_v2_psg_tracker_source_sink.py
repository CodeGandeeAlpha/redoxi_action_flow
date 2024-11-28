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

psg_tracker_node_pipeline_json_params = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {"buffer_capacity": 10, "action_name": "in/action"},
        "output_port_pipeline_config": {
            "downstream_specs": [
                {
                    "name": "document_sink",
                    "action_name": "/document_sink/in/action",
                }
            ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "any_downstream_ready",
        },
        "output_port_model_config": {
            "downstream_specs": [
                {
                    "name": "psg_tracker_node",
                    "action_name": "/psg_tracker_node/in/action",
                }
            ],
        },
        "create_debug_pub": False,
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": 5,
        "frame_interval": 0,
        "enable_blocking_mode": False,
        "publish_to_debug_topic": False,
        "frame_request_policy": {
            "precondition": "any_downstream_ready",
            "drop_strategy": "no_drop",
        },
    },
}

psg_tracker_node_json_params = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {"buffer_capacity": 10, "action_name": "in/action"},
        "create_debug_pub": False,
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": 5,
        "frame_interval": 0,
        "enable_blocking_mode": False,
        "enable_debug_topics": False,
        "publish_to_debug_topic": False,
        "frame_request_policy": {
            "precondition": "any_downstream_ready",
            "drop_strategy": "no_drop",
        },
    },
}

document_sink_node_json_params = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {"buffer_capacity": 10, "action_name": "in/action"},
        "_time_unit": "us(1e-6)",
        "step_interval": 500,
        "publish_topic": "out/relayed_document",
    },
    "runtime_config": {
        "enable_debug_topics": True,
        "enable_blocking_mode": False,
    },
}

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []


psg_tracker_pipeline_node = Node(
    package="test_cx",
    executable="v2_psg_tracker_pipeline",
    name="psg_tracker_pipeline_node",
    namespace="psg_tracker_pipeline_node",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_tracker_node_pipeline_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["psg_tracker_pipeline_node:=", logger]]
    + common_ros_args,
)

psg_tracker_node = Node(
    package="test_cx",
    executable="v2_psg_tracker",
    name="psg_tracker_node",
    namespace="psg_tracker_node",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_tracker_node_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["psg_tracker_node:=", logger]]
    + common_ros_args,
)

document_sink_node = Node(
    package="test_cx",
    executable="v2_document_sink",
    name="document_sink",
    namespace="document_sink",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                document_sink_node_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["document_sink:=", logger]]
    + common_ros_args,
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
            psg_tracker_pipeline_node,
            psg_tracker_node,
            document_sink_node,
        ]
    )
