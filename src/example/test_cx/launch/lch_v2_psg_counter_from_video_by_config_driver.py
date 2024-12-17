from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import psg_common_py.configs.psg_document_sink as psgDocSinkCfg
import psg_common_py.configs.driver_base as psgDriverBaseCfg
import psg_common_py.configs.inout_base as psgInoutBaseCfg
import psg_common_py.configs.psg_counter as psgCounterCfg
import psg_common_py.configs.psg_tracker as psgTrackerCfg

import json

logger = LaunchConfiguration("log_level")
log_level_arg = DeclareLaunchArgument(
    "log_level",
    default_value="debug",
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
psg_counter_node_json_params = psgCounterCfg.PSGCounterNodeConfig(
    init_config=psgCounterCfg.PSGCounterInitConfig(
        create_debug_pub=True,
        passengerflow_config_path="/3d/chengxiao/data/passengerflow/fairmot_train_230907/psg_configs/20.22.6.30.json",
        input_port_config=psgCounterCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_config=psgCounterCfg.OutputPortConfig(
            downstream_specs=[
                psgCounterCfg.DownstreamSpec(
                    name=document_sink_node_name,
                    action_name=f"/{document_sink_node_name}/{document_sink_node_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data",
        ),
    ),
    runtime_config=psgCounterCfg.PSGCounterRuntimeConfig(
        step_interval=StepIntervals.VeryFast,
        document_interval=0,
        enable_blocking_mode=False,
        publish_to_debug_topic=False,
        frame_request_policy=psgCounterCfg.DeliveryPolicy(
            precondition="no_precondition",
            drop_strategy="no_drop",
        ),
        frame_enqueue_policy=psgCounterCfg.DeliveryPolicy(
            precondition="no_precondition",
            drop_strategy="no_drop",
        ),
    ),
)


class TrackerType:
    DEEPSORT = 0
    BOTSORT = 1


# PSG跟踪器节点配置
# psg_tracker_pipeline_node <-> psg_tracker_node
psg_tracker_node_name = "psg_tracker_node"
psg_tracker_node_json_params = psgTrackerCfg.PSGTrackerNodeConfig(
    init_config=psgTrackerCfg.PSGTrackerInitConfig(
        tracker_type=TrackerType.BOTSORT,
        input_port_config=psgTrackerCfg.InputPortConfig(
            action_name="in/action",
        ),
    ),
    runtime_config=psgTrackerCfg.PSGTrackerRuntimeConfig(
        enable_debug_topics=False,
        enable_blocking_mode=False,
    ),
)

# PSG跟踪器节点配置
# psg_tracker_driver -> psg_tracker_node
psg_tracker_node_driver_name = "psg_tracker_driver"
psg_tracker_node_driver_json_params = psgDriverBaseCfg.DriverBaseNodeConfig(
    init_config=psgDriverBaseCfg.DriverBaseInitConfig(
        input_port_config=psgDriverBaseCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_config=psgDriverBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgDriverBaseCfg.DownstreamSpec(
                    name=psg_counter_node_name,
                    action_name=f"/{psg_counter_node_name}/{psg_counter_node_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data_pipeline",
        ),
        callee_request_port_config=psgDriverBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgDriverBaseCfg.DownstreamSpec(
                    name=psg_tracker_node_name,
                    action_name=f"/{psg_tracker_node_name}/{psg_tracker_node_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data_model",
        ),
    ),
    runtime_config=psgDriverBaseCfg.DriverBaseRuntimeConfig(
        enable_blocking_mode=False,
    ),
)

# PSG人员生成器节点配置
# psg_all_detector_cpp -> psg_person_generator -> psg_tracker_pipeline_node
psg_person_generator_node_name = "psg_person_generator"
psg_person_generator_node_json_params = psgInoutBaseCfg.InoutBaseNodeConfig(
    init_config=psgInoutBaseCfg.InoutBaseInitConfig(
        create_debug_pub=True,
        input_port_config=psgInoutBaseCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_config=psgInoutBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgInoutBaseCfg.DownstreamSpec(
                    name=psg_tracker_node_driver_name,
                    action_name=f"/{psg_tracker_node_driver_name}/{psg_tracker_node_driver_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data",
        ),
    ),
    runtime_config=psgInoutBaseCfg.InoutBaseRuntimeConfig(
        step_interval=StepIntervals.VeryFast,
        document_interval=0,
        enable_blocking_mode=False,
        publish_to_debug_topic=False,
        frame_request_policy=psgInoutBaseCfg.DeliveryPolicy(
            precondition="no_precondition",
            drop_strategy="no_drop",
        ),
        frame_enqueue_policy=psgInoutBaseCfg.DeliveryPolicy(
            precondition="no_precondition",
            drop_strategy="no_drop",
        ),
    ),
)

# PSG Pose Detector 节点配置
# psg_pose_detector_driver <-> psg_pose_detector_node_model
psg_pose_detector_node_driver_name = "psg_pose_detector_driver"
psg_pose_detector_node_driver_json_params = psgDriverBaseCfg.DriverBaseNodeConfig(
    init_config=psgDriverBaseCfg.DriverBaseInitConfig(
        input_port_config=psgDriverBaseCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_config=psgDriverBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgDriverBaseCfg.DownstreamSpec(
                    name=psg_person_generator_node_name,
                    action_name=f"/{psg_person_generator_node_name}/{psg_person_generator_node_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data_pipeline",
        ),
        callee_request_port_config=psgDriverBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgDriverBaseCfg.DownstreamSpec(
                    name="rtm_pose_detector_node",
                    action_name=f"/rtm_pose_detector_node/model_process_detections_action",
                )
            ],
            data_topic_for_target_data="data_out/target_data_model",
        ),
    ),
    runtime_config=psgDriverBaseCfg.DriverBaseRuntimeConfig(
        enable_blocking_mode=False,
    ),
)

# PSG Detector 节点配置
# psg_detector_driver -> psg_detector_node
psg_detector_node_driver_name = "psg_detector_driver"
psg_detector_node_driver_json_params = psgDriverBaseCfg.DriverBaseNodeConfig(
    init_config=psgDriverBaseCfg.DriverBaseInitConfig(
        input_port_config=psgDriverBaseCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_config=psgDriverBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgDriverBaseCfg.DownstreamSpec(
                    name=psg_pose_detector_node_driver_name,
                    action_name=f"/{psg_pose_detector_node_driver_name}/{psg_pose_detector_node_driver_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data_pipeline",
        ),
        callee_request_port_config=psgDriverBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgDriverBaseCfg.DownstreamSpec(
                    name="psg_detector",
                    action_name=f"/psg_detector/model_process_frame_action",
                )
            ],
            data_topic_for_target_data="data_out/target_data_model",
        ),
    ),
    runtime_config=psgDriverBaseCfg.DriverBaseRuntimeConfig(
        enable_blocking_mode=False,
    ),
)

# PSG主节点配置
# psg_master_node -> psg_detector_node
psg_master_node_name = "psg_master_node"
psg_master_node_json_params = psgInoutBaseCfg.InoutBaseNodeConfig(
    init_config=psgInoutBaseCfg.InoutBaseInitConfig(
        create_debug_pub=True,
        input_port_config=psgInoutBaseCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_config=psgInoutBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgInoutBaseCfg.DownstreamSpec(
                    name=psg_detector_node_driver_name,
                    action_name=f"/{psg_detector_node_driver_name}/{psg_detector_node_driver_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data",
        ),
    ),
    runtime_config=psgInoutBaseCfg.InoutBaseRuntimeConfig(
        step_interval=StepIntervals.VeryFast,
        document_interval=0,
        enable_blocking_mode=False,
        publish_to_debug_topic=False,
        frame_request_policy=psgInoutBaseCfg.DeliveryPolicy(
            precondition="no_precondition",
            drop_strategy="no_drop",
        ),
        frame_enqueue_policy=psgInoutBaseCfg.DeliveryPolicy(
            precondition="no_precondition",
            drop_strategy="no_drop",
        ),
    ),
)


# 视频源配置
# video_source -> psg_master_node
video_src_node_name = "video_source"
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
            data_topic_for_target_data="data_out/target_data",
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=StepIntervals.Medium,
        output_image_size=videoSrcCfg.ImageSize(width=1920, height=1080),
        output_image_encoding="bgr8",
    ),
)


# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

video_source_node = Node(
    package="test_cx",
    executable="v2_video_url_flush",
    name=video_src_node_name,
    namespace=video_src_node_name,
    output="screen",
    parameters=[
        {
            "param_as_json_string": video_source_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    prefix=common_prefix,
    arguments=["--ros-args", "--log-level", [f"{video_src_node_name}:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
)


psg_master_node = Node(
    package="test_cx",
    executable="v2_psg_master_node",
    output="screen",  # 将输出重定向到屏幕
    emulate_tty=True,  # 保持颜色输出
    name=psg_master_node_name,
    namespace=psg_master_node_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_master_node_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{psg_master_node_name}:=", logger]]
    + common_ros_args,
)

psg_detector_driver_node = Node(
    package="test_cx",
    executable="v2_psg_detector_driver",
    name=psg_detector_node_driver_name,
    namespace=psg_detector_node_driver_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_detector_node_driver_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_detector_node_driver_name}:=", logger],
    ]
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

psg_pose_detector_driver_node = Node(
    package="test_cx",
    executable="v2_psg_pose_detector_driver",
    name=psg_pose_detector_node_driver_name,
    namespace=psg_pose_detector_node_driver_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_pose_detector_node_driver_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_pose_detector_node_driver_name}:=", logger],
    ]
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
    name=psg_person_generator_node_name,
    namespace=psg_person_generator_node_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_person_generator_node_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_person_generator_node_name}:=", logger],
    ]
    + common_ros_args,
)

psg_tracker_driver_node = Node(
    package="test_cx",
    executable="v2_psg_tracker_driver",
    name=psg_tracker_node_driver_name,
    namespace=psg_tracker_node_driver_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_tracker_node_driver_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_tracker_node_driver_name}:=", logger],
    ]
    + common_ros_args,
)

psg_tracker_node = Node(
    package="test_cx",
    executable="v2_psg_tracker",
    name=psg_tracker_node_name,
    namespace=psg_tracker_node_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_tracker_node_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{psg_tracker_node_name}:=", logger]]
    + common_ros_args,
)

psg_counter_node = Node(
    package="test_cx",
    executable="v2_psg_counter",
    output="screen",  # 将输出重定向到屏幕
    emulate_tty=True,  # 保持颜色输出
    name=psg_counter_node_name,
    namespace=psg_counter_node_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_counter_node_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{psg_counter_node_name}:=", logger]]
    + common_ros_args,
)

document_sink_node = Node(
    package="test_cx",
    executable="v2_document_sink",
    name=document_sink_node_name,
    namespace=document_sink_node_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": document_sink_node_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{document_sink_node_name}:=", logger]]
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
            psg_detector_driver_node,
            psg_detector_node_model,
            psg_pose_detector_driver_node,
            psg_pose_detector_node_model,
            psg_person_generator_node,
            psg_tracker_driver_node,
            psg_tracker_node,
            psg_counter_node,
            document_sink_node,
        ]
    )
