try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av

from redoxi_common_py.configs.async_ports import *
import redoxi_common_py.configs.async_ports as portCfg

from redoxi_common_py.configs.base_node import *
import redoxi_common_py.configs.base_node as baseNodeCfg
from typing import Any, Literal, ClassVar

ExampleConfig = {
    "declare_params": {},
    "init_config": {
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

__all__ = (
    [
        "VideoReaderBaseInitConfig",
        "VideoReaderBaseRuntimeConfig",
    ]
    + baseNodeCfg.__all__
    + portCfg.__all__
)


@define(kw_only=True)
class VideoReaderBaseInitConfig(baseNodeCfg.BaseRosNodeInitConfig):
    # action of the primary output port
    PrimaryOutputGoalType: ClassVar[str] = "redoxi_public_msgs/action/ProcessFrame_Goal"

    primary_output_spec: OutputPortConfig | None = field(default=None)
    create_debug_pub: bool = field(default=True)
    debug_pub_task_enqueue_name: str | None = field(default=None)
    debug_pub_task_drop_name: str | None = field(default=None)


@define(kw_only=True)
class VideoReaderBaseRuntimeConfig(baseNodeCfg.BaseRosNodeRuntimeConfig):
    output_image_size: ImageSize = field(factory=ImageSize)
    output_image_encoding: str = field(
        default=baseNodeCfg.DefaultSettings.image_encoding
    )
    publish_to_debug_topic: bool = field(default=False)

    # this will affect how the frame is enqueued to the primary output port
    frame_enqueue_policy: portCfg.DeliveryPolicy | None = field(default=None)

    # this will override all downstream policies
    frame_request_policy: portCfg.DeliveryPolicy | None = field(default=None)
