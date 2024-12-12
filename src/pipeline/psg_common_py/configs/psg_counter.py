import redoxi_common_py.configs.async_ports as portCfg
from redoxi_common_py.configs.async_ports import *

import psg_common_py.configs.inout_base as inoutBaseCfg
from psg_common_py.configs.inout_base import *

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
        "PipelineBaseInitConfig",
        "PipelineBaseRuntimeConfig",
    ]
    + commonTypes.__all__
    + portCfg.__all__
    + baseNodeCfg.__all__
)


@define(kw_only=True)
class PSGCounterInitConfig(inoutBaseCfg.InoutBaseInitConfig):
    passengerflow_config_path: str = field(default="")


@define(kw_only=True)
class PSGCounterRuntimeConfig(inoutBaseCfg.InoutBaseRuntimeConfig):
    pass


@define(kw_only=True)
class InoutBaseNodeConfig(inoutBaseCfg.InoutBaseNodeConfig):
    init_config: PSGCounterInitConfig = field(default=None)
    runtime_config: PSGCounterRuntimeConfig = field(default=None)
