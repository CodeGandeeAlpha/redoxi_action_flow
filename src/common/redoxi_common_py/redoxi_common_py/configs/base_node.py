try:
    from attrs import define, field
    import attrs.validators as av
except ImportError:
    from attr import define, field
    import attr.validators as av
from typing import Literal, Dict, Any
import redoxi_common_py.common_types as commonTypes
from redoxi_common_py.common_types import *

__all__ = [
    "BaseRosNodeInitConfig",
    "BaseRosNodeRuntimeConfig",
    "BaseRosNodeConfig",
    "ModelConfig",
] + commonTypes.__all__


@define(kw_only=True)
class BaseRosNodeInitConfig(commonTypes.JsonConvertible):
    _time_unit: RedoxiTimeUnit = field(default=DefaultSettings.time_unit)


@define(kw_only=True)
class BaseRosNodeRuntimeConfig(commonTypes.JsonConvertible):
    _time_unit: RedoxiTimeUnit = field(default=DefaultSettings.time_unit)
    step_interval: int = field(default=DefaultSettings.step_interval)


@define(kw_only=True)
class BaseRosNodeConfig(commonTypes.JsonConvertible):
    # these will be defined as independent ROS node parameters
    declare_params: Dict[str, Any] = field(factory=dict)


@define(kw_only=True)
class ModelConfig(commonTypes.JsonConvertible):
    model_path: str = field()
    device_type: Literal["cpu", "cuda"] = field(validator=av.in_(["cpu", "cuda"]))
    device_index: int = field(validator=av.ge(0))
