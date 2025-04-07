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

from typing import Any, Literal

ExampleConfig = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {
            "_action_goal_type": "redoxi_public_msgs/action/ProcessDetections_Goal",
            "buffer_capacity": -1,
            "action_name": "",
            "goal_result_expire_time": 1000000,
        },
        "publish_detection_topic": "out/relayed_detection",
        "publish_visualization_topic": "out/relayed_visualization",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "enable_blocking_mode": False,
        "enable_visualization": True,
        "_time_unit": "us(1e-6)",
        "step_interval": 5000,
    },
}

__all__ = (
    [
        "DetectionRelayInitConfig",
        "DetectionRelayRuntimeConfig",
        "DetectionRelayNodeConfig",
    ]
    + baseNodeCfg.__all__
    + portCfg.__all__
)


@define(kw_only=True)
class DetectionRelayInitConfig(baseNodeCfg.BaseRosNodeInitConfig):
    # action type: redoxi_public_msgs/action/ProcessDetections_Goal
    input_port_config: portCfg.InputPortConfig | None = field(default=None)
    publish_detection_topic: str | None = field(default=None)
    publish_visualization_topic: str | None = field(default=None)


@define(kw_only=True)
class DetectionRelayRuntimeConfig(baseNodeCfg.BaseRosNodeRuntimeConfig):
    enable_blocking_mode: bool = field(default=False)
    enable_visualization: bool = field(default=True)


@define(kw_only=True)
class DetectionRelayNodeConfig(baseNodeCfg.BaseRosNodeConfig):
    init_config: DetectionRelayInitConfig = field(factory=DetectionRelayInitConfig)
    runtime_config: DetectionRelayRuntimeConfig = field(
        factory=DetectionRelayRuntimeConfig
    )
