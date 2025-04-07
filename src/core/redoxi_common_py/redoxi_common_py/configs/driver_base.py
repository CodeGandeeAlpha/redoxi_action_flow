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
            "_action_goal_type": "redoxi_public_msgs/action/ProcessFrame_Goal",
            "buffer_capacity": -1,
            "action_name": "",
            "goal_result_expire_time": 1000000,
        },
        "output_port_config": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessDetections_Goal",
            "downstream_specs": [],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "dont_care",
        },
        "callee_request_port_config": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessDetectionsByFrame_Goal",
            "downstream_specs": [],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "dont_care",
        },
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "callee_request_enqueue_policy": {
            "retry_policy": {
                "fallback_number_of_retry": 3,
                "fallback_wait_time_between_retry": 5000,
                "fallback_wait_time_retry_response": 100000,
            },
            "precondition": "dont_care",
            "drop_strategy": "dont_care",
        },
        "driver_output_enqueue_policy": {
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
        "DriverBaseInitConfig",
        "DriverBaseRuntimeConfig",
    ]
    + commonTypes.__all__
    + portCfg.__all__
    + baseNodeCfg.__all__
)


@define(kw_only=True)
class DriverBaseInitConfig(baseNodeCfg.BaseRosNodeInitConfig):
    input_port_config: portCfg.InputPortConfig | None = field(default=None)
    output_port_config: portCfg.OutputPortConfig | None = field(default=None)
    callee_request_port_config: portCfg.OutputPortConfig | None = field(default=None)


@define(kw_only=True)
class DriverBaseRuntimeConfig(baseNodeCfg.BaseRosNodeRuntimeConfig):
    callee_request_enqueue_policy: portCfg.DeliveryPolicy = field(default=None)
    driver_output_enqueue_policy: portCfg.DeliveryPolicy = field(default=None)
    enable_blocking_mode: bool = field(default=False)
