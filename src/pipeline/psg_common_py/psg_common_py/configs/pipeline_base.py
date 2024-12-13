import redoxi_common_py.configs.async_ports as portCfg
from redoxi_common_py.configs.async_ports import *

import redoxi_common_py.configs.base_node as baseNodeCfg
from redoxi_common_py.configs.base_node import *

import redoxi_common_py.common_types as commonTypes
from redoxi_common_py.common_types import *

try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av

from typing import Any, Literal

# this is detection driver
ExampleConfig = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {
            "_action_goal_type": "psg_private_msgs/action/ProcessDocument_Goal",
            "buffer_capacity": -1,
            "action_name": "",
            "goal_result_expire_time": 1000000,
        },
        "output_port_pipeline_config": {
            "_action_goal_type": "psg_private_msgs/action/ProcessDocument_Goal",
            "downstream_specs": [],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "dont_care",
        },
        "output_port_model_config": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessDetectionByFrame_Goal",
            "downstream_specs": [],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "dont_care",
        },
    },
    "runtime_config": {
        "frame_request_policy": {
            "retry_policy": {
                "fallback_number_of_retry": 3,
                "fallback_wait_time_between_retry": 5000,
                "fallback_wait_time_retry_response": 100000,
            },
            "precondition": "dont_care",
            "drop_strategy": "dont_care",
        },
        "frame_enqueue_policy": {
            "retry_policy": {
                "fallback_number_of_retry": 3,
                "fallback_wait_time_between_retry": 5000,
                "fallback_wait_time_retry_response": 100000,
            },
            "precondition": "dont_care",
            "drop_strategy": "dont_care",
        },
        "enable_blocking_mode": False,
        "_time_unit": "us(1e-6)",
        "step_interval": 5000,
    },
}

__all__ = (
    [
        "PipelineBaseInitConfig",
        "PipelineBaseRuntimeConfig",
        "PipelineBaseNodeConfig",
    ]
    + commonTypes.__all__
    + portCfg.__all__
    + baseNodeCfg.__all__
)


@define(kw_only=True)
class PipelineBaseInitConfig(baseNodeCfg.BaseRosNodeInitConfig):
    input_port_config: portCfg.InputPortConfig | None = field(default=None)
    output_port_pipeline_config: portCfg.OutputPortConfig | None = field(default=None)
    output_port_model_config: portCfg.OutputPortConfig | None = field(default=None)
    create_debug_pub: bool = field(default=False)
    debug_pub_queue_size: int = field(default=10)
    debug_pub_pipeline_enqueue_name: str = field(default="debug_port/pipeline_enqueue")
    debug_pub_pipeline_drop_name: str = field(default="debug_port/pipeline_drop")
    debug_pub_model_enqueue_name: str = field(default="debug_port/model_enqueue")
    debug_pub_model_drop_name: str = field(default="debug_port/model_drop")


@define(kw_only=True)
class PipelineBaseRuntimeConfig(baseNodeCfg.BaseRosNodeRuntimeConfig):
    pipeline_enqueue_policy: portCfg.DeliveryPolicy = field(default=None)
    model_enqueue_policy: portCfg.DeliveryPolicy = field(default=None)
    pipeline_request_policy: portCfg.DeliveryPolicy = field(default=None)
    model_request_policy: portCfg.DeliveryPolicy = field(default=None)
    enable_blocking_mode: bool = field(default=False)
    publish_to_debug_topic: bool = field(default=False)
    document_interval: int = field(default=0)


@define(kw_only=True)
class PipelineBaseNodeConfig(baseNodeCfg.BaseRosNodeConfig):
    init_config: PipelineBaseInitConfig = field(default=None)
    runtime_config: PipelineBaseRuntimeConfig = field(default=None)
