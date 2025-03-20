# launch simple video reader node
# make sure you have PYTHONPATH set currectly, you can use the following command to set it:
# env | grep PYTHONPATH
# and then copy the PYTHONPATH to .env file

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument

import redoxi_common_py.configs.video_source_from_url as videoSrcCfg
import redoxi_common_py.configs.frame_relay as frameRelayCfg


class RunConfig:
    WorkspaceRoot = "/soft/workspace/code/psf_ros2_ws"
    LogLevel = "info"
    JsonParamKey = "param_as_json_string"  # used by ros nodes


logger = LaunchConfiguration("log_level")
log_level_arg = DeclareLaunchArgument(
    "log_level",
    default_value=RunConfig.LogLevel,
    description="Logging level",
)


def get_logger_arg(node_name: str):
    return ["--ros-args", "--log-level", [f"{node_name}:=", logger]]


# the pipeline structure is:
# video_source -> frame_relay

frame_relay_node_config = frameRelayCfg.FrameRelayNodeConfig(
    init_config=frameRelayCfg.FrameRelayNodeInitConfig(
        # the upstream should send goals to this action server
        input_port_config=frameRelayCfg.InputPortConfig(
            action_name="input_frame",
        ),
        # the frames got from upstream will be sent to this topic for visualization
        publish_topic="relayed_frame",
    ),
    runtime_config=frameRelayCfg.FrameRelayNodeRuntimeConfig(
        step_interval=1000 * 1000 // 5,  # pop input every this many macroseconds
    ),
)

frame_relay_node = Node(
    package="redoxi_samples_nodes",
    executable="node_pack_samples_FrameRelayNode",
    name="my_frame_relay",
    namespace="my_frame_relay",
    parameters=[
        {
            RunConfig.JsonParamKey: frame_relay_node_config.to_json(
                ignore_none=True, compact=False
            )
        }
    ],
    arguments=get_logger_arg("my_frame_relay"),
)

video_source_node_config = videoSrcCfg.VideoSourceFromUrlNodeConfig(
    init_config=videoSrcCfg.VideoSourceFromUrlInitConfig(
        video_url=f"{RunConfig.WorkspaceRoot}/data/dancetrack/dancetrack-0039.mp4",
        # optional published topics for monitoring
        primary_output_spec=videoSrcCfg.OutputPortConfig(
            downstream_specs=[
                videoSrcCfg.DownstreamSpec(
                    name="relay_downstream",  # can be anything
                    action_name="/my_frame_relay/input_frame",  # absolute action name of downstream input port
                )
            ],
            data_topic_for_source_data=f"data_msg/source_data",  # get reliably published data for visualization
            visualization_topic_for_target_data=f"vis/target_data",  # get lossy published data for visualization
            probe_topic_for_target_data=f"probe/target_data",  # get reliable published string for probing
        ),
    ),
    runtime_config=videoSrcCfg.VideoSourceFromUrlRuntimeConfig(
        step_interval=1000 // 25 * 1000,  # time in macroseconds (1e-6 s)
        # optional, resize the frame to this size
        output_image_size={"width": 1024, "height": -1},
        # retry policy, specify how to handle transmission failures
        # if not set, by default it will retry as many times as possible until success.
        # the frame data will be first enqueued to the reader node's sending buffer, and then send to downstream
        # a frame may be dropped in the following cases:
        # 1. in enqueue stage, if sending buffer is full because sending is too slow
        # 2. in delivery stage, if downstream is not responsive or rejects the frame
        # so, we have 2 retry policies, one for enqueue stage, one for delivery stage
        frame_enqueue_policy=videoSrcCfg.DeliveryPolicy(
            drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
            retry_policy=videoSrcCfg.RetryPolicy(
                number_of_retry=3,
                wait_time_between_retry=1000 * 10,  # in 1e-6 s
            ),
        ),
        frame_request_policy=videoSrcCfg.DeliveryPolicy(
            drop_strategy=videoSrcCfg.DropStrategy.DropAsNeeded,
            retry_policy=videoSrcCfg.RetryPolicy(
                number_of_retry=3,
                wait_time_between_retry=1000 * 10,  # in 1e-6 s
                wait_time_retry_response=1000 * 1000,  # in 1e-6 s
            ),
        ),
    ),
)

# executable name can be found in the CMakeLists.txt file of the target package
video_source_node = Node(
    package="redoxi_video_reader",
    executable="node_pack_video_readers_VideoSourceFromUrl",
    name="my_video_source",
    namespace="my_video_source",
    parameters=[
        {
            RunConfig.JsonParamKey: video_source_node_config.to_json(
                ignore_none=True, compact=False
            )
        }
    ],
    arguments=get_logger_arg("my_video_source"),
)


# Add lifecycle manager to handle transitions
lifecycle_manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager",
    output="screen",
    parameters=[
        {
            "node_names": [
                "/my_video_source/my_video_source",
                "/my_frame_relay/my_frame_relay",
            ],
            "bond_timeout": 0.0,
            "autostart": True,  # This will automatically configure and activate nodes
        },
    ],
)


def generate_launch_description():
    # Set environment variables for ROS 2 logging format
    env_var_settings = [
        SetEnvironmentVariable(
            "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}][{time}]: {message}"
        ),
        SetEnvironmentVariable("ROS_LOG_DIR", f"{RunConfig.WorkspaceRoot}/tmp/roslog"),
    ]

    return LaunchDescription(
        [
            *env_var_settings,
            log_level_arg,
            lifecycle_manager,
            video_source_node,
            frame_relay_node,
        ]
    )
