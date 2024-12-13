from redoxi_common_py.configs.async_ports import *
import redoxi_common_py.configs.async_ports as portCfg

from redoxi_common_py.configs.base_node import *
import redoxi_common_py.configs.base_node as baseNodeCfg
from typing import Any, Literal, ClassVar

try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av

ExampleConfig = {
    "declare_params": {},
    "init_config": {
        "input_port_config": {
            "_action_goal_type": "private_msgs/action/ProcessDocument_Goal",
            "buffer_capacity": 1,
            "action_name": "in/frame",
            "goal_result_expire_time": 1000000,
        },
        "publish_topic": "out/relayed_frame",
        "debug_topic_frame_accepted": "debug/frame_accepted",
        "debug_topic_frame_rejected": "debug/frame_rejected",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "enable_blocking_mode": False,
        "enable_debug_topics": True,
        "_time_unit": "us(1e-6)",
        "step_interval": 5000,
    },
}

__all__ = (
    [
        "PSGDocumentSinkNodeConfig",
        "PSGDocumentSinkNodeInitConfig",
        "PSGDocumentSinkNodeRuntimeConfig",
    ]
    + portCfg.__all__
    + baseNodeCfg.__all__
)


@define(kw_only=True)
class PSGDocumentSinkNodeInitConfig(baseNodeCfg.BaseRosNodeInitConfig):
    input_port_config: portCfg.InputPortConfig | None = field(default=None)
    publish_topic: str | None = field(default=None)
    debug_topic_frame_accepted: str | None = field(default=None)
    debug_topic_frame_rejected: str | None = field(default=None)


@define(kw_only=True)
class PSGDocumentSinkNodeRuntimeConfig(baseNodeCfg.BaseRosNodeRuntimeConfig):
    enable_blocking_mode: bool = field(default=False)
    enable_debug_topics: bool = field(default=False)


@define(kw_only=True)
class PSGDocumentSinkNodeConfig(baseNodeCfg.BaseRosNodeConfig):
    init_config: PSGDocumentSinkNodeInitConfig = field(
        factory=PSGDocumentSinkNodeInitConfig
    )
    runtime_config: PSGDocumentSinkNodeRuntimeConfig = field(
        factory=PSGDocumentSinkNodeRuntimeConfig
    )
