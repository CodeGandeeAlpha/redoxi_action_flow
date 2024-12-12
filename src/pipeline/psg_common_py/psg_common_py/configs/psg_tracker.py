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

__all__ = (
    [
        "PSGTrackerInitConfig",
        "PSGTrackerRuntimeConfig",
        "PSGTrackerNodeConfig",
    ]
    + commonTypes.__all__
    + portCfg.__all__
    + baseNodeCfg.__all__
)


@define(kw_only=True)
class PSGTrackerInitConfig(baseNodeCfg.BaseRosNodeInitConfig):
    input_port_config: portCfg.InputPortConfig | None = field(default=None)
    create_debug_pub: bool = field(default=False)
    debug_pub_queue_size: int = field(default=10)
    debug_topic_person_accepted: str = field(default="debug_port/person_accepted")
    debug_topic_person_rejected: str = field(default="debug_port/person_rejected")
    tracker_type: int = field(default=0)


@define(kw_only=True)
class PSGTrackerRuntimeConfig(baseNodeCfg.BaseRosNodeRuntimeConfig):
    enable_blocking_mode: bool = field(default=False)
    enable_debug_topics: bool = field(default=True)


@define(kw_only=True)
class PSGTrackerNodeConfig(baseNodeCfg.BaseRosNodeConfig):
    init_config: PSGTrackerInitConfig = field(default=None)
    runtime_config: PSGTrackerRuntimeConfig = field(default=None)
