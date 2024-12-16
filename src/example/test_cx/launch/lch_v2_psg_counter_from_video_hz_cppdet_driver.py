from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import psg_common_py.configs.psg_document_sink as psgDocSinkCfg
import psg_common_py.configs.pipeline_base as psgPipelineBaseCfg
import psg_common_py.configs.driver_base as psgDriverBaseCfg
import psg_common_py.configs.inout_base as psgInoutBaseCfg
import psg_common_py.configs.psg_counter as psgCounterCfg
import psg_common_py.configs.psg_tracker as psgTrackerCfg
import yolo8_series.configs as yolo
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
        # step_interval=StepIntervals.VeryFast,
        document_interval=0,
        enable_blocking_mode=False,
        publish_to_debug_topic=False,
        # frame_request_policy=psgCounterCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
        # frame_enqueue_policy=psgCounterCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
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
# psg_tracker_pipeline_node -> psg_tracker_node
psg_tracker_node_pipeline_name = "psg_tracker_pipeline_node"
psg_tracker_node_pipeline_json_params = psgPipelineBaseCfg.PipelineBaseNodeConfig(
    init_config=psgPipelineBaseCfg.PipelineBaseInitConfig(
        create_debug_pub=True,
        input_port_config=psgPipelineBaseCfg.InputPortConfig(
            action_name="in/action",
        ),
        output_port_pipeline_config=psgPipelineBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgPipelineBaseCfg.DownstreamSpec(
                    name=psg_counter_node_name,
                    action_name=f"/{psg_counter_node_name}/{psg_counter_node_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data_pipeline",
        ),
        output_port_model_config=psgPipelineBaseCfg.OutputPortConfig(
            downstream_specs=[
                psgPipelineBaseCfg.DownstreamSpec(
                    name=psg_tracker_node_name,
                    action_name=f"/{psg_tracker_node_name}/{psg_tracker_node_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data_model",
        ),
    ),
    runtime_config=psgPipelineBaseCfg.PipelineBaseRuntimeConfig(
        publish_to_debug_topic=False,
        enable_blocking_mode=False,
        # pipeline_enqueue_policy=psgPipelineBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
        # pipeline_request_policy=psgPipelineBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
        # model_enqueue_policy=psgPipelineBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
        # model_request_policy=psgPipelineBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
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
                    name=psg_tracker_node_pipeline_name,
                    action_name=f"/{psg_tracker_node_pipeline_name}/{psg_tracker_node_pipeline_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data",
        ),
    ),
    runtime_config=psgInoutBaseCfg.InoutBaseRuntimeConfig(
        # step_interval=StepIntervals.VeryFast,
        document_interval=0,
        enable_blocking_mode=False,
        publish_to_debug_topic=False,
        # frame_request_policy=psgInoutBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
        # frame_enqueue_policy=psgInoutBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
    ),
)

# cpp 姿态检测节点配置
# psg_all_detector_cpp <-> yolo_body_pose_detection_node
fn_model_nano = "/3d/chengxiao/code/psf_ros2_ws/tmp/models/yolov8n-pose-640.onnx"
det_node_name = "yolo_body_pose_detection_node"
det_node_params = yolo.Yolo8ModelNodeConfig(
    init_config=yolo.Yolo8ModelInitConfig(
        model_configs=[
            {
                "model_path": fn_model_nano,
                "device_type": "cuda",
                "device_index": 0,
            },
        ],
        detection_request_config=yolo.DetectionRequestConfig(
            input_port_config=yolo.InputPortConfig(
                action_name="in/detection_request",
            ),
        ),
        publish_visualization_topic="debug/visualization",
        publish_probe_detection_done_topic="probe/detection_done",
    ),
    runtime_config=yolo.Yolo8ModelRuntimeConfig(
        model_output_config=yolo.ModelPostprocessConfig(
            conf_threshold=0.2,
            iou_threshold=0.4,
        ),
        enable_blocking_mode=False,
        enable_performance_probe=True,
        enable_visualization=True,
    ),
)

# PSG cpp 姿态检测pipeline节点配置
# psg_all_detector_cpp <-> yolo_body_pose_detection_node
psg_all_detector_cpp_node_driver_name = "psg_all_detector_cpp_driver"
psg_all_detector_cpp_node_driver_json_params = psgDriverBaseCfg.DriverBaseNodeConfig(
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
                    name=det_node_name,
                    action_name=f"/{det_node_name}/{det_node_params.init_config.detection_request_config.input_port_config.action_name}",
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
# psg_master_node -> psg_all_detector_cpp_node_pipeline
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
                    name=psg_all_detector_cpp_node_driver_name,
                    action_name=f"/{psg_all_detector_cpp_node_driver_name}/{psg_all_detector_cpp_node_driver_json_params.init_config.input_port_config.action_name}",
                )
            ],
            data_topic_for_target_data="data_out/target_data",
        ),
    ),
    runtime_config=psgInoutBaseCfg.InoutBaseRuntimeConfig(
        # step_interval=StepIntervals.VeryFast,
        document_interval=0,
        enable_blocking_mode=False,
        publish_to_debug_topic=False,
        # frame_request_policy=psgInoutBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
        # frame_enqueue_policy=psgInoutBaseCfg.DeliveryPolicy(
        #     precondition="no_precondition",
        #     drop_strategy="no_drop",
        # ),
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
                    create_debug_pub=True,
                ),
            ],
            data_topic_for_target_data="data_out/target_data",
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=StepIntervals.Fast,
        output_image_size=videoSrcCfg.ImageSize(width=1024, height=-1),
        output_image_encoding="bgr8",
        frame_request_policy=videoSrcCfg.DeliveryPolicy(
            drop_strategy="drop_as_needed",
        ),
        frame_enqueue_policy=videoSrcCfg.DeliveryPolicy(
            drop_strategy="drop_as_needed",
        ),
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

psg_detector_node = Node(
    package="test_cx",
    executable="v2_psg_all_detector_cpp_driver",
    name=psg_all_detector_cpp_node_driver_name,
    namespace=psg_all_detector_cpp_node_driver_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_all_detector_cpp_node_driver_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_all_detector_cpp_node_driver_name}:=", logger],
    ]
    + common_ros_args,
)

detection_node = Node(
    package="test_package",
    executable="yolo_body_pose_detection_node",
    name=det_node_name,
    namespace=det_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": det_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=["--ros-args", "--log-level", [f"{det_node_name}:=", logger]]
    + common_ros_args,
    # arguments=["--ros-args", "--disable-external-lib-logs"],
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

psg_tracker_pipeline_node = Node(
    package="test_cx",
    executable="v2_psg_tracker_pipeline",
    name=psg_tracker_node_pipeline_name,
    namespace=psg_tracker_node_pipeline_name,
    prefix=common_prefix,
    parameters=[
        {
            "param_as_json_string": psg_tracker_node_pipeline_json_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_tracker_node_pipeline_name}:=", logger],
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
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_tracker_node_name}:=", logger],
    ]
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
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{psg_counter_node_name}:=", logger],
    ]
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
    arguments=[
        "--ros-args",
        "--log-level",
        [f"{document_sink_node_name}:=", logger],
    ]
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
