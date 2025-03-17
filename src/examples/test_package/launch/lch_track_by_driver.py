from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
import os
import json

import yolo8_series.configs as yolo
import redoxi_common_py.configs.detection_relay as detRelayCfg
import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import redoxi_common_py.configs.detection_driver as detDriverCfg
import redoxi_common_py.configs.frame_relay as frameRelayCfg
import universal_mot_trackers.node_configs as motTrackersCfg
import universal_mot_trackers.driver_configs as motTrackersDriverCfg

# Get workspace root from COLCON_PREFIX_PATH
import os

workspace_root = os.getenv("COLCON_PREFIX_PATH", "")
if workspace_root:
    # Remove /install from the end if present
    if workspace_root.endswith("/install"):
        workspace_root = workspace_root[:-8]  # Remove last 8 characters ('/install')


try:
    from attrs import asdict
except ImportError:
    from attr import asdict

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


class InputCacheSize:
    Small = 1
    Medium = 2
    Large = 4


# the graph structure is:
# video_source -> detection_driver -> tracker_driver -> frame_relay
#                       |                   |
#                    detector           tracker

# tracker_driver -> frame_relay
frame_relay_node_name = "frame_relay"
frame_relay_node_params = frameRelayCfg.FrameRelayNodeConfig(
    init_config=frameRelayCfg.FrameRelayNodeInitConfig(
        input_port_config=frameRelayCfg.InputPortConfig(
            action_name="in/frame",
            buffer_capacity=InputCacheSize.Medium,
        ),
        publish_topic="out/relayed_frame",
    ),
)

# tracker_driver <-> tracker
tracker_node_name = "tracker"
tracker_node_params = motTrackersCfg.UniversalMotTrackersNodeConfig(
    init_config=motTrackersCfg.UniversalMotTrackersInitConfig(
        input_port_config=motTrackersCfg.InputPortConfig(
            action_name="in/track_request",
            buffer_capacity=InputCacheSize.Medium,
        ),
        # publish_visualization_topic="vis/tracking",
        # preferred_image_size={"width": 1920, "height": 1080},
    ),
    runtime_config=motTrackersCfg.UniversalMotTrackersRuntimeConfig(
        enable_blocking_mode=False,
        enable_visualization=False,
        enable_performance_probe=True,
    ),
)

# tracker_driver -> frame_relay
#       |
#    tracker
tracker_driver_node_name = "tracker_driver"
tracker_driver_node_params = motTrackersDriverCfg.TrackerDriverNodeConfig(
    init_config=motTrackersDriverCfg.TrackerDriverInitConfig(
        input_port_config=motTrackersDriverCfg.InputPortConfig(
            action_name="in/detections",
            buffer_capacity=InputCacheSize.Medium,
        ),
        output_port_config=motTrackersDriverCfg.OutputPortConfig(
            # downstream_specs=[
            #     motTrackersDriverCfg.DownstreamSpec(
            #         name=frame_relay_node_name,
            #         action_name=f"/{frame_relay_node_name}/{frame_relay_node_params.init_config.input_port_config.action_name}",
            #         create_debug_pub=True,
            #     ),
            # ],
            data_topic_for_source_data="output/data/source_data",  # source data msg is image, transmission is reliable, better smoothness in visualization
            # visualization_topic_for_source_data="vis_msg/source_data",  # source visualization msg is image, but transmission is unreliable, may drop frames
            # data_topic_for_target_data="data_msg/target_data",  # target data msg is tracking result, transmission is reliable, can be used for data processing
            visualization_topic_for_target_data="output/vis/target_data",  # target visualization msg is image, but transmission is unreliable, may drop frames
            probe_topic_for_target_data="output/probe/target_data",
            probe_topic_for_source_data="output/probe/source_data",
        ),
        callee_request_port_config=motTrackersDriverCfg.OutputPortConfig(
            downstream_specs=[
                motTrackersDriverCfg.DownstreamSpec(
                    name=tracker_node_name,
                    action_name=f"/{tracker_node_name}/{tracker_node_params.init_config.input_port_config.action_name}",
                ),
            ],
            probe_topic_for_source_data="callee/probe/source_data",
            probe_topic_for_target_data="callee/probe/target_data",
        ),
    ),
)


# detection_driver <-> detector
# fn_model = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8s.onnx"
# fn_model = r"/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8m-pose-dynbatch.onnx"
# fn_model = f"{workspace_root}/tmp/models/yolov8n-pose-640.onnx"
fn_model = f"{workspace_root}/tmp/models/yolov8s-pose.onnx"
# fn_model = (
#     f"{workspace_root}/tmp/models/rknn/pose/small/yolov8s-pose-224x384-bs1-pthq.rknn"
# )
# fn_model = f"/data/code/psf_ros2_ws/tmp/models/rknn/pose/nano/yolov8n-pose-352x640-bs1-pthq.rknn"
det_node_name = "detector"
det_node_params = yolo.Yolo8ModelNodeConfig(
    init_config=yolo.Yolo8ModelInitConfig(
        model_configs=[
            {
                "model_path": fn_model,
                "device_type": "cuda",
                "device_index": 0,
            },
            # {
            #     "model_path": fn_model,
            #     "device_type": "rknpu",
            #     "device_index": -2,
            # },
        ],
        detection_request_config=yolo.DetectionRequestConfig(
            input_port_config=yolo.InputPortConfig(
                action_name="in/detection_request",
                buffer_capacity=InputCacheSize.Medium,
            ),
        ),
    ),
    runtime_config=yolo.Yolo8ModelRuntimeConfig(
        model_output_config=yolo.ModelPostprocessConfig(
            conf_threshold=0.2,
            iou_threshold=0.6,
            selected_class_ids=[0],  # 0=person (ultralytics convention)
        )
    ),
)

# detection_driver -> tracker_driver
#       |
#    detector
det_driver_node_name = "detection_driver"
det_driver_node_params = detDriverCfg.DetectionDriverNodeConfig(
    init_config=detDriverCfg.DetectionDriverInitConfig(
        input_port_config=detDriverCfg.InputPortConfig(
            action_name="in/frame",
            buffer_capacity=InputCacheSize.Medium,
        ),
        output_port_config=detDriverCfg.OutputPortConfig(
            downstream_specs=[
                detDriverCfg.DownstreamSpec(
                    name=tracker_driver_node_name,
                    action_name=f"/{tracker_driver_node_name}/{tracker_driver_node_params.init_config.input_port_config.action_name}",
                    # create_debug_pub=True,
                ),
            ],
            # visualization_topic_for_source_data="output/vis/source_data",
            # visualization_topic_for_target_data="output/vis/target_data",
            probe_topic_for_target_data="output/probe/target_data",
            probe_topic_for_source_data="output/probe/source_data",
        ),
        callee_request_port_config=detDriverCfg.OutputPortConfig(
            downstream_specs=[
                detDriverCfg.DownstreamSpec(
                    name=det_node_name,
                    action_name=f"/{det_node_name}/{det_node_params.init_config.detection_request_config.input_port_config.action_name}",
                    # create_debug_pub=True,
                ),
            ],
            # visualization_topic_for_source_data="vis/callee/source_data",
            # visualization_topic_for_target_data="callee/vis/target_data",
            probe_topic_for_target_data="callee/probe/target_data",
            probe_topic_for_source_data="callee/probe/source_data",
        ),
    ),
    runtime_config=detDriverCfg.DetectionDriverRuntimeConfig(
        enable_blocking_mode=False,
    ),
)

# video_source -> detection_driver
video_src_node_name = "video_source"
# fn_video = f"{workspace_root}/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4"
fn_video = f"{workspace_root}/.bigdata/crowded_0820.coded.mp4"
# fn_video = f"{workspace_root}/.bigdata/crowded_0820.mp4"
# fn_video = f"{workspace_root}/.bigdata/new-york.mp4"
# fn_video = f"{workspace_root}/data/dancetrack/dancetrack-0039.mp4"
video_src_node_params = videoSrcCfg.VideoSourceFromUrlNodeConfig(
    init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
        video_url=fn_video,
        auto_replay=True,
        primary_output_spec=videoSrcCfg.OutputPortConfig(
            downstream_specs=[
                videoSrcCfg.DownstreamSpec(
                    name=det_driver_node_name,
                    action_name=f"/{det_driver_node_name}/{det_driver_node_params.init_config.input_port_config.action_name}",
                    # create_debug_pub=True,
                ),
            ],
            # data_topic_for_source_data="data_msg/source_data",
            # data_topic_for_target_data="data_msg/target_data",
            probe_topic_for_target_data="probe/target_data",
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=StepIntervals.Medium,
        video_start_time=0,
        video_end_time=-1,
        frame_enqueue_policy=videoSrcCfg.DeliveryPolicy(
            precondition=videoSrcCfg.DeliveryPrecondition.DontCare,
            drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
        ),
        frame_request_policy=videoSrcCfg.DeliveryPolicy(
            precondition=videoSrcCfg.DeliveryPrecondition.DontCare,
            drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
            retry_policy=videoSrcCfg.RetryPolicy(
                number_of_retry=0,
                wait_time_between_retry=1000,
                wait_time_retry_response=1000,
            ),
        ),
        # output_image_size={"width": 1920, "height": 1080},
        output_image_size={"width": 1024, "height": -1},
    ),
)

# common_prefix = ["valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no"]
common_prefix = None
# common_ros_args = ["--disable-external-lib-logs"]
common_ros_args = []

frame_relay_node = Node(
    package="test_package",
    executable="frame_relay_node",
    name=frame_relay_node_name,
    namespace=frame_relay_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": frame_relay_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

det_driver_node = Node(
    package="test_package",
    executable="detection_driver_node",
    name=det_driver_node_name,
    namespace=det_driver_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": det_driver_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

video_src_node = Node(
    package="test_package",
    executable="video_from_url_node",
    name=video_src_node_name,
    namespace=video_src_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": video_src_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

detection_node = Node(
    package="test_package",
    # executable="yolo_object_detection_node",
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

tracking_node = Node(
    package="test_package",
    executable="mot_tracker_node",
    name=tracker_node_name,
    namespace=tracker_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": tracker_node_params.to_json(
                ignore_none=True, compact=False
            ),
        },
    ],
)

tracking_driver_node = Node(
    package="test_package",
    executable="mot_tracker_driver",
    name=tracker_driver_node_name,
    namespace=tracker_driver_node_name,
    prefix=common_prefix,
    output="screen",
    parameters=[
        {
            "param_as_json_string": tracker_driver_node_params.to_json(
                ignore_none=True, compact=False
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
        SetEnvironmentVariable("ROS_LOG_DIR", f"{workspace_root}/tmp/roslog"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            detection_node,
            video_src_node,
            det_driver_node,
            tracking_node,
            tracking_driver_node,
            # frame_relay_node,
        ]
    )
