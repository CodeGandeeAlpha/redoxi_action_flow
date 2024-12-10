from attrs import define, field, asdict
import attrs.validators as av
from typing import Literal
from redoxi_common_py.common_types import *
from datetime import timedelta


@define(kw_only=True)
class BaseRosNodeInitConfig:
    _time_unit: RedoxiTimeUnit = field(factory=RedoxiTimeUnit.Default)


@define(kw_only=True)
class BaseRosNodeRuntimeConfig:
    _time_unit: RedoxiTimeUnit = field(factory=RedoxiTimeUnit.Default)
    step_interval: int = field(factory=lambda: int(timedelta(microseconds=5000)))
