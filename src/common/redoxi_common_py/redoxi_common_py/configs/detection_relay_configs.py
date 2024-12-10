try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av

import redoxi_common_py.configs.async_port_configs as portConfig
import redoxi_common_py.configs.node_configs as nodeConfig
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


@define(kw_only=True)
class InitConfig(nodeConfig.BaseRosNodeInitConfig):
    input_port_config: portConfig.InputPortConfig | None = field(default=None)
    publish_detection_topic: str | None = field(default=None)
    publish_visualization_topic: str | None = field(default=None)


@define(kw_only=True)
class RuntimeConfig(nodeConfig.BaseRosNodeRuntimeConfig):
    enable_blocking_mode: bool = field(default=False)
    enable_visualization: bool = field(default=True)


@define(kw_only=True)
class DetectionRelayNodeConfig(nodeConfig.JsonConvertible):
    declare_params: dict[str, Any] | None = field(default=None)
    init_config: InitConfig = field(factory=InitConfig)
    runtime_config: RuntimeConfig = field(factory=RuntimeConfig)
