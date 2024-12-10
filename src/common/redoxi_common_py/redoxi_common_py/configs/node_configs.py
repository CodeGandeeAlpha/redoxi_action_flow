try:
    from attrs import define, field
    import attrs.validators as av
except ImportError:
    from attr import define, field
    import attr.validators as av
from typing import Literal
from redoxi_common_py.common_types import *

__all__ = ["JsonConvertible", "BaseRosNodeInitConfig", "BaseRosNodeRuntimeConfig"]


@define(kw_only=True)
class BaseRosNodeInitConfig(JsonConvertible):
    _time_unit: RedoxiTimeUnit = field(default=DefaultSettings.time_unit)


@define(kw_only=True)
class BaseRosNodeRuntimeConfig(JsonConvertible):
    _time_unit: RedoxiTimeUnit = field(default=DefaultSettings.time_unit)
    step_interval: int = field(default=DefaultSettings.step_interval)


@define(kw_only=True)
class ModelConfig(JsonConvertible):
    model_path: str = field()
    device_type: Literal["cpu", "cuda"] = field(validator=av.in_(["cpu", "cuda"]))
    device_index: int = field(validator=av.ge(0))
