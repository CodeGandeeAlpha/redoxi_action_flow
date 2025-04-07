import redoxi_common_py.configs.driver_base as driverCfg
from redoxi_common_py.configs.driver_base import *

import redoxi_common_py.configs.async_ports as portCfg
from redoxi_common_py.configs.async_ports import *

try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av
from typing import Dict, Any, ClassVar

ExampleConfig = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessDetections_Goal",
            "buffer_capacity": -1,
            "action_name": "in/tracking_request",
            "goal_result_expire_time": 1000000,
        },
        "output_port_config": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessFrame_Goal",
            "downstream_specs": [],
            "num_buffer_requests": 1,
            "preserve_request_order": True,
            "fallback_delivery_precondition": "dont_care",
        },
        "callee_request_port_config": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessTrackByDetection_Goal",
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
                "fallback_wait_time_retry_response": 1000000,
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
        "TrackerDriverInitConfig",
        "TrackerDriverRuntimeConfig",
        "TrackerDriverNodeConfig",
    ]
    + driverCfg.__all__
    + portCfg.__all__
)


@define(kw_only=True)
class TrackerDriverInitConfig(driverCfg.DriverBaseInitConfig):
    """Initialization configuration for TrackerDriverNode.

    action goal types
    --------------------
    input_port_config:
    - redoxi_public_msgs/action/ProcessDetections_Goal

    output_port_config:
    - redoxi_public_msgs/action/ProcessFrame_Goal

    callee_request_port_config:
    - redoxi_public_msgs/action/ProcessTrackByDetection_Goal
    """

    pass


@define(kw_only=True)
class TrackerDriverRuntimeConfig(driverCfg.DriverBaseRuntimeConfig):
    pass


@define(kw_only=True)
class TrackerDriverNodeConfig(driverCfg.BaseRosNodeConfig):
    init_config: TrackerDriverInitConfig = field(default=None)
    runtime_config: TrackerDriverRuntimeConfig = field(default=None)
