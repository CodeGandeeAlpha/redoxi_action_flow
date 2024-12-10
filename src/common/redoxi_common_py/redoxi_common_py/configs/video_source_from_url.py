try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av

# from redoxi_common_py.configs.base_node import *

from redoxi_common_py.configs.video_reader_base import *
import redoxi_common_py.configs.video_reader_base as videoReaderBaseCfg

from typing import Any, Literal, ClassVar

ExampleConfig = {
    "declare_params": {},
    "init_config": {
        "video_url": "",
        "auto_replay": False,
        "primary_output_spec": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessFrame_Goal",
            "downstream_specs": [
                {
                    "name": "",
                    "action_name": "",
                    "delivery_policy": {
                        "retry_policy": {
                            "fallback_number_of_retry": 3,
                            "fallback_wait_time_between_retry": 5000,
                            "fallback_wait_time_retry_response": 1000000,
                        },
                        "precondition": "dont_care",
                        "drop_strategy": "dont_care",
                    },
                    "create_debug_pub": False,
                }
            ],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "dont_care",
        },
        "create_debug_pub": True,
        "debug_pub_queue_size": 10,
        "debug_pub_task_enqueue_name": "debug_port/task_enqueue",
        "debug_pub_task_drop_name": "debug_port/task_drop",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "video_start_time": 0,
        "video_end_time": -1,
        "frame_interval": 0,
        "output_image_size": {"width": -1, "height": -1},
        "output_image_encoding": "rgb8",
        "publish_to_debug_topic": False,
        "frame_enqueue_policy": {
            "retry_policy": {
                "number_of_retry": 5,
                "fallback_number_of_retry": 3,
                "wait_time_between_retry": 5000,
                "fallback_wait_time_between_retry": 5000,
                "wait_time_retry_response": 5000,
                "fallback_wait_time_retry_response": 1000000,
            },
            "precondition": "any_downstream_ready",
            "drop_strategy": "drop_as_needed",
        },
        "_time_unit": "us(1e-6)",
        "step_interval": 5000,
    },
}

__all__ = [
    "VideoSourceFromUrlInitConfig",
    "VideoSourceFromUrlRuntimeConfig",
    "VideoSourceFromUrlNodeConfig",
] + videoReaderBaseCfg.__all__


@define(kw_only=True)
class VideoSourceFromUrlInitConfig(videoReaderBaseCfg.VideoReaderBaseInitConfig):
    video_url: str = field()
    auto_replay: bool = field(default=False)


@define(kw_only=True)
class VideoSourceFromUrlRuntimeConfig(videoReaderBaseCfg.VideoReaderBaseRuntimeConfig):
    # start from this time, in _time_unit
    video_start_time: int = field(default=0)
    # end at this time, in _time_unit, -1 means no end time
    video_end_time: int = field(default=-1)


@define(kw_only=True)
class VideoSourceFromUrlNodeConfig(videoReaderBaseCfg.JsonConvertible):
    declare_params: dict[str, Any] = field(factory=dict)
    init_config: VideoSourceFromUrlInitConfig = field(
        factory=VideoSourceFromUrlInitConfig
    )
    runtime_config: VideoSourceFromUrlRuntimeConfig = field(
        factory=VideoSourceFromUrlRuntimeConfig
    )
