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

psg_detector_node_pipeline_json_params = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {"buffer_capacity": 10, "action_name": "in/action"},
        "output_port_pipeline_config": {
            "downstream_specs": [
                {
                    "name": "pose_detector_node",
                    "action_name": "/pose_detector_node/in/action",
                }
            ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "any_downstream_ready",
        },
        "output_port_model_config": {
            "downstream_specs": [
                {
                    "name": "psg_detector",
                    "action_name": "/psg_detector/model_process_frame_action",
                }
            ],
        },
        "create_debug_pub": True,
        "_time_unit": "us(1e-6)",
        "enable_blocking_mode": False,
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": 10000000,
        "frame_interval": 0,
        "publish_to_debug_topic": False,
        "frame_request_policy": {
            "precondition": "any_downstream_ready",
            "drop_strategy": "no_drop",
        },
    },
}

psg_pose_detector_node_pipeline_json_params = {
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
                    "name": "rtm_pose_detector_node",
                    "action_name": "/rtm_pose_detector_node/model_process_detections_action",
                }
            ],
        },
        "create_debug_pub": True,
        "_time_unit": "us(1e-6)",
        "enable_blocking_mode": False,
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": 5,
        "frame_interval": 0,
        "publish_to_debug_topic": True,
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
        "enable_debug_topics": True,
        "enable_blocking_mode": False,
    },
}

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []


psg_detector_node = Node(
    package="test_cx",
    executable="v2_psg_detector",
    name="detector_node",
    namespace="detector_node",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_detector_node_pipeline_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["psg_detector:=", logger]]
    + common_ros_args,
)

psg_detector_node_model = Node(
    package="psg_detector",
    executable="ddq_detector_node.py",
    name="psg_detector",
    namespace="psg_detector",
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", ["psg_detector:=", logger]]
    + common_ros_args,
)

psg_pose_detector_node = Node(
    package="test_cx",
    executable="v2_psg_pose_detector",
    name="pose_detector_node",
    namespace="pose_detector_node",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_pose_detector_node_pipeline_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["pose_detector_node:=", logger]]
    + common_ros_args,
)

psg_pose_detector_node_model = Node(
    package="psg_pose_detector",
    executable="rtm_pose_detector_node.py",
    name="rtm_pose_detector_node",
    namespace="rtm_pose_detector_node",
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", ["rtm_pose_detector_node:=", logger]]
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
            psg_detector_node,
            psg_detector_node_model,
            psg_pose_detector_node,
            psg_pose_detector_node_model,
            document_sink_node,
        ]
    )
