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

fn_video = (
    "/3d/chengxiao/code/psf_ros2_ws/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4"
)

# 定义一些通用的配置
default_retry_policy = {
    "fallback_number_of_retry": 3,
    "fallback_wait_time_between_retry": 5000,
    "fallback_wait_time_retry_response": 100000,
}

default_delivery_policy = {
    "retry_policy": default_retry_policy,
    "drop_strategy": "no_drop",
}

default_enqueue_policy = {
    "retry_policy": {"number_of_retry": 5, **default_retry_policy},
    # "precondition": "no_precondition",
    # "drop_strategy": "no_drop",
    "drop_strategy": "drop_as_needed",
}

default_request_policy = {"precondition": "no_precondition", "drop_strategy": "no_drop"}

default_output_port_config = {
    "num_buffer_requests": 1,
    "preserve_request_order": True,
    "fallback_delivery_precondition": "no_precondition",
}

default_init_config = {
    "create_debug_pub": True,
    "_time_unit": "us(1e-6)",
    "input_port_config": {"buffer_capacity": 10, "action_name": "in/action"},
}

default_runtime_config = {
    "_time_unit": "us(1e-6)",
    "step_interval": 5,
    "frame_interval": 0,
    "enable_blocking_mode": False,
    "publish_to_debug_topic": False,
}

# 视频源配置
video_source_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "video_url": fn_video,
        "auto_replay": True,
        "primary_output_spec": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "psg_master_node",
                    "action_name": "/psg_master_node/in/action",
                    "delivery_policy": default_delivery_policy,
                    "create_debug_pub": False,
                }
            ],
        },
        "create_debug_pub": False,
        "debug_pub_queue_size": 10,
        "debug_pub_task_enqueue_name": "debug_port/task_enqueue",
        "debug_pub_task_drop_name": "debug_port/task_drop",
    },
    "runtime_config": {
        **default_runtime_config,
        "video_start_time": 0,
        "video_end_time": -1,
        "output_image_size": {"width": 1024, "height": -1},
        "output_image_encoding": "rgb8",
        "publish_to_debug_topic": True,
        "frame_enqueue_policy": default_enqueue_policy,
        "frame_request_policy": default_request_policy,
        "step_interval": 50000,
    },
}

# PSG主节点配置
psg_master_node_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "output_port_config": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "psg_detector",
                    "action_name": "/detector_node/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
        },
        "create_debug_pub": False,
    },
    "runtime_config": {
        **default_runtime_config,
        "frame_enqueue_policy": default_enqueue_policy,
        "frame_request_policy": default_request_policy,
    },
}

# PSG检测器节点配置
psg_detector_node_pipeline_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "output_port_pipeline_config": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "pose_detector_node",
                    "action_name": "/pose_detector_node/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
        },
        "output_port_model_config": {
            "downstream_specs": [
                {
                    "name": "psg_detector",
                    "action_name": "/psg_detector/model_process_frame_action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
            "fallback_delivery_precondition": "no_precondition",
        },
    },
    "runtime_config": {
        **default_runtime_config,
        "pipeline_enqueue_policy": default_enqueue_policy,
        "pipeline_request_policy": default_request_policy,
        "model_enqueue_policy": default_enqueue_policy,
        "model_request_policy": default_request_policy,
    },
}

# PSG姿态检测节点配置
psg_pose_detector_node_pipeline_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "output_port_pipeline_config": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "psg_person_generator",
                    "action_name": "/psg_person_generator/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
        },
        "output_port_model_config": {
            "downstream_specs": [
                {
                    "name": "rtm_pose_detector_node",
                    "action_name": "/rtm_pose_detector_node/model_process_detections_action",
                    "delivery_policy": default_delivery_policy,
                }
            ]
        },
    },
    "runtime_config": {
        **default_runtime_config,
        "frame_enqueue_policy": default_enqueue_policy,
        "frame_request_policy": default_request_policy,
    },
}

# PSG人员生成器节点配置
psg_person_generator_node_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "output_port_config": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "psg_tracker_pipeline_node",
                    "action_name": "/psg_tracker_pipeline_node/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
        },
    },
    "runtime_config": {
        **default_runtime_config,
        "pipeline_enqueue_policy": default_enqueue_policy,
        "pipeline_request_policy": default_request_policy,
        "model_enqueue_policy": default_enqueue_policy,
        "model_request_policy": default_request_policy,
    },
}

# PSG跟踪器节点配置
psg_tracker_node_pipeline_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "output_port_pipeline_config": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "document_sink",
                    "action_name": "/document_sink/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
        },
        "output_port_model_config": {
            "downstream_specs": [
                {
                    "name": "psg_tracker_node",
                    "action_name": "/psg_tracker_node/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ]
        },
    },
    "runtime_config": {
        **default_runtime_config,
        "pipeline_enqueue_policy": default_enqueue_policy,
        "pipeline_request_policy": default_request_policy,
        "model_enqueue_policy": default_enqueue_policy,
        "model_request_policy": default_request_policy,
    },
}

# PSG跟踪器节点配置
psg_tracker_node_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "create_debug_pub": False,
        "tracker_type": 0,
    },
    "runtime_config": {**default_runtime_config, "enable_debug_topics": False},
}

# 文档接收节点配置
document_sink_node_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "step_interval": 5,
        "publish_topic": "out/relayed_document",
    },
    "runtime_config": {"enable_debug_topics": True, "enable_blocking_mode": False},
}

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

video_source_node = Node(
    package="test_cx",
    executable="v2_video_url_flush",
    name="video_source",
    namespace="video_source",
    output="screen",
    parameters=[
        {
            "param_as_json_string": json.dumps(
                video_source_params, separators=(",", ":")
            ),
        },
    ],
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", [f"video_source:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
)


psg_master_node = Node(
    package="test_cx",
    executable="v2_psg_master_node",
    output="screen",  # 将输出重定向到屏幕
    emulate_tty=True,  # 保持颜色输出
    name="psg_master_node",
    namespace="psg_master_node",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_master_node_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["psg_master_node:=", logger]]
    + common_ros_args,
)

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
    arguments=["--ros-args", "--log-level", ["detector_node:=", logger]]
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

psg_person_generator_node = Node(
    package="test_cx",
    executable="v2_psg_person_generator",
    output="screen",  # 将输出重定向到屏幕
    emulate_tty=True,  # 保持颜色输出
    name="psg_person_generator",
    namespace="psg_person_generator",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_person_generator_node_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["psg_person_generator:=", logger]]
    + common_ros_args,
)

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
            video_source_node,
            psg_master_node,
            psg_detector_node,
            psg_detector_node_model,
            psg_pose_detector_node,
            psg_pose_detector_node_model,
            psg_person_generator_node,
            psg_tracker_pipeline_node,
            psg_tracker_node,
            document_sink_node,
        ]
    )
