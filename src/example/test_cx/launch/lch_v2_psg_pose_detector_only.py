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

psg_pose_detector_node_pipeline_json_params = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {"buffer_capacity": 10, "action_name": "in/action"},
        "output_port_pipeline_config": {
            # "downstream_specs": [
            #     {
            #         "name": "psg_person_generator",
            #         "action_name": "/psg_person_generator/in/action",
            #     }
            # ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "any_downstream_ready",
        },
        "output_port_model_config": {
            # "downstream_specs": [
            #     {
            #         "name": "rtm_pose_detector_node",
            #         "action_name": "/rtm_pose_detector_node/model_process_detections_action",
            #     }
            # ],
        },
        "create_debug_pub": True,
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

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []


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

# psg_pose_detector_node_model = Node(
#     package="psg_pose_detector",
#     executable="rtm_pose_detector_node.py",
#     name="rtm_pose_detector_node",
#     namespace="rtm_pose_detector_node",
#     prefix=common_prefix,
#     arguments=["--ros-args", "--log-level", ["rtm_pose_detector_node:=", logger]]
#     + common_ros_args,
# )


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
            psg_pose_detector_node,
        ]
    )
