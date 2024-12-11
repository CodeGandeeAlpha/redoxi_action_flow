import redoxi_common_py.configs.base_node as nodeCfg
from redoxi_common_py.configs.base_node import *

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
            "_action_goal_type": "redoxi_public_msgs/action/ProcessTrackByDetection_Goal",
            "buffer_capacity": -1,
            "action_name": "in/tracking_request",
            "goal_result_expire_time": 1000000,
        },
        "publish_visualization_topic": "out/visualization",
        "publish_probe_topic": "out/probe",
        "preferred_image_size": {"width": 1920, "height": 1080},
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "enable_blocking_mode": False,
        "enable_visualization": True,
        "enable_performance_probe": True,
        "_time_unit": "us(1e-6)",
        "step_interval": 5000,
    },
}

__all__ = (
    [
        "UniversalMotTrackersInitConfig",
        "UniversalMotTrackersRuntimeConfig",
        "UniversalMotTrackersConfig",
    ]
    + nodeCfg.__all__
    + portCfg.__all__
)


@define(kw_only=True)
class UniversalMotTrackersInitConfig(nodeCfg.BaseRosNodeInitConfig):
    InputPortActionGoalType: ClassVar[str] = (
        "redoxi_public_msgs/action/ProcessTrackByDetection_Goal"
    )
    input_port_config: portCfg.InputPortConfig | None = field(default=None)
    publish_visualization_topic: str | None = field(default=None)
    publish_probe_topic: str | None = field(default=None)
    preferred_image_size: ImageSize | None = field(default=None)


@define(kw_only=True)
class UniversalMotTrackersRuntimeConfig(nodeCfg.BaseRosNodeRuntimeConfig):
    enable_blocking_mode: bool = field(default=False)
    enable_visualization: bool = field(default=True)
    enable_performance_probe: bool = field(default=True)


@define(kw_only=True)
class UniversalMotTrackersNodeConfig(nodeCfg.BaseRosNodeConfig):
    init_config: UniversalMotTrackersInitConfig = field(
        factory=UniversalMotTrackersInitConfig
    )
    runtime_config: UniversalMotTrackersRuntimeConfig = field(
        factory=UniversalMotTrackersRuntimeConfig
    )
