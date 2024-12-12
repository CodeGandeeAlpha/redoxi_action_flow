from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import pipeline.psg_common_py.configs.psg_document_sink as psgDocSinkCfg
import pipeline.psg_common_py.configs.pipeline_base as psgPipelineBaseCfg
import pipeline.psg_common_py.configs.inout_base as psgInoutBaseCfg
import json

logger = LaunchConfiguration("log_level")
log_level_arg = DeclareLaunchArgument(
    "log_level",
    default_value="info",
    description="Logging level",
)


class StepIntervals:
    VerySlow = 3000000
    Slow = 200000
    Medium = 1000000 / 25
    Fast = 5000
    VeryFast = 1000


fn_video = (
    # "/3d/chengxiao/code/psf_ros2_ws/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4"
    "/3d/chengxiao/data/passengerflow/fairmot_train_230907/videos/20.22.6.30-2023-06-18-15-00-02.mp4"
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

# 文档接收节点配置
# psg_counter -> document_sink
document_sink_node_name = "document_sink"
document_sink_node_json_params = psgDocSinkCfg.PSGDocumentSinkNodeConfig(
    init_config=psgDocSinkCfg.PSGDocumentSinkNodeInitConfig(
        input_port_config=psgDocSinkCfg.InputPortConfig(
            action_name="in/action",
        ),
        publish_topic="out/relayed_document",
    ),
    runtime_config=psgDocSinkCfg.PSGDocumentSinkNodeRuntimeConfig(
        enable_debug_topics=True,
        enable_blocking_mode=False,
    ),
)

# PSG计数器节点配置
# psg_tracker_pipeline_node -> psg_counter
psg_counter_node_name = "psg_counter"
psg_counter_node_json_params = psgInoutBaseCfg.InoutBaseNodeConfig(
    init_config=psgInoutBaseCfg.InoutBaseInitConfig(
        create_debug_pub=True,
        input_port_config=psgInoutBaseCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_config=psgInoutBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgInoutBaseCfg.DownstreamSpec(
                    name="document_sink",
                    action_name="/document_sink/in/action",
                    delivery_policy=default_delivery_policy,
                )
            ],
        ),
    ),
)
psg_counter_node_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "output_port_config": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "document_sink",
                    "action_name": "/document_sink/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
        },
        "passengerflow_config_path": "/3d/chengxiao/data/passengerflow/fairmot_train_230907/psg_configs/20.22.6.30.json",
    },
    "runtime_config": {
        **default_runtime_config,
        "pipeline_enqueue_policy": default_enqueue_policy,
        "pipeline_request_policy": default_request_policy,
        "model_enqueue_policy": default_enqueue_policy,
        "model_request_policy": default_request_policy,
    },
}


# 视频源配置
video_source_params = videoSrcCfg.VideoSourceFromUrlNodeConfig(
    init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
        video_url=fn_video,
        auto_replay=True,
        primary_output_spec=videoSrcCfg.OutputPortConfig(
            downstream_specs=[
                videoSrcCfg.DownstreamSpec(
                    name=psg_master_node_name,
                    action_name=f"/{psg_master_node_name}/{psg_master_node_json_params.init_config.input_port_config.action_name}",
                ),
            ],
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=StepIntervals.Medium,
        output_image_size={"width": 1920, "height": 1080},
        output_image_encoding="bgr8",
    ),
)

# PSG主节点配置
psg_master_node_name = "psg_master_node"
psg_master_node_json_params = {
    "declare_params": {},
    "init_config": {
        **default_init_config,
        "output_port_config": {
            **default_output_port_config,
            "downstream_specs": [
                {
                    "name": "psg_all_detector_cpp",
                    "action_name": "/psg_all_detector_cpp/in/action",
                    "delivery_policy": default_delivery_policy,
                }
            ],
        },
        "create_debug_pub": True,
        "debug_pub_queue_size": 10,
        "debug_pub_task_enqueue_name": "debug_port/task_enqueue",
        "debug_pub_task_drop_name": "debug_port/task_drop",
    },
    "runtime_config": {
        **default_runtime_config,
        "frame_enqueue_policy": default_enqueue_policy,
        "frame_request_policy": default_request_policy,
    },
}

# PSG cpp 姿态检测pipeline节点配置
psg_all_detector_cpp_node_pipeline_json_params = {
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
                    "name": "detector_with_pose",
                    "action_name": "/detector_with_pose/in/detection_request",
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

fn_model_nano = "/3d/chengxiao/code/psf_ros2_ws/tmp/models/yolov8n-pose-640.onnx"
det_node_params = {
    "declare_params": {},
    "init_config": {
        "model_configs": [
            {
                "model_path": fn_model_nano,
                "device_type": "cuda",
                "device_index": 0,
            },
        ],
        "detection_request_config": {
            "input_port_config": {
                "buffer_capacity": 1,
                "action_name": "in/detection_request",
                "goal_result_expire_time": 1000000,
            }
        },
        "publish_visualization_topic": "debug/visualization",
        "publish_probe_detection_done_topic": "probe/detection_done",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "_time_unit": "us(1e-6)",
        "step_interval": 1000,
        "enable_blocking_mode": False,
        "enable_performance_probe": True,
        "model_output_config": {"conf_threshold": 0.1, "iou_threshold": 0.6},
        "enable_visualization": True,
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
                    "name": "psg_counter",
                    "action_name": "/psg_counter/in/action",
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
        "tracker_type": 1,
    },
    "runtime_config": {**default_runtime_config, "enable_debug_topics": False},
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
    executable="v2_psg_all_detector_cpp",
    name="psg_all_detector_cpp",
    namespace="psg_all_detector_cpp",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_all_detector_cpp_node_pipeline_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["psg_all_detector_cpp:=", logger]]
    + common_ros_args,
)

DetectionNodeName = "detector_with_pose"
detection_node = Node(
    package="test_package",
    executable="yolo_body_pose_detection_node",
    name=DetectionNodeName,
    namespace=DetectionNodeName,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": json.dumps(det_node_params, separators=(",", ":")),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{DetectionNodeName}:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
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

psg_counter_node = Node(
    package="test_cx",
    executable="v2_psg_counter",
    output="screen",  # 将输出重定向到屏幕
    emulate_tty=True,  # 保持颜色输出
    name="psg_counter",
    namespace="psg_counter",
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": json.dumps(
                psg_counter_node_json_params, separators=(",", ":")
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", ["psg_counter:=", logger]]
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
            detection_node,
            psg_person_generator_node,
            psg_tracker_pipeline_node,
            psg_tracker_node,
            psg_counter_node,
            document_sink_node,
        ]
    )
