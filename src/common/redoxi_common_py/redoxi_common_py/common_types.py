from enum import Enum
import json

try:
    from attrs import define, field, asdict
except ImportError:
    from attr import define, field, asdict

__all__ = [
    "RedoxiTimeUnit",
    "DropStrategy",
    "DeliveryPrecondition",
    "DefaultSettings",
    "JsonConvertible",
]


@define(kw_only=True)
class JsonConvertible:
    def to_json(self, ignore_none: bool = False, compact: bool = False):
        """
        Convert the object to a JSON string.

        parameters
        -----------
        ignore_none: bool, default=True
            If True, ignore None values in the output JSON.
        compact: bool, default=True
            If True, use compact JSON format with no spaces.

        returns
        -------
        str
            The JSON string representation of the object.
        """
        if ignore_none:
            data = asdict(self, filter=lambda attr, value: value is not None)
        else:
            data = asdict(self)
        separators = (",", ":") if compact else None
        indent = 2 if not compact else None
        return json.dumps(data, separators=separators, indent=indent)


# default to microseconds
class RedoxiTimeUnit(str, Enum):
    Hours = "hours"
    Minutes = "minutes"
    Seconds = "seconds"
    Milliseconds = "ms(1e-3)"
    Microseconds = "us(1e-6)"
    Nanoseconds = "ns(1e-9)"


class DropStrategy(str, Enum):
    DontCare = "dont_care"
    NoDrop = "no_drop"
    DropAsNeeded = "drop_as_needed"


class DeliveryPrecondition(str, Enum):
    DontCare = "dont_care"
    NoPrecondition = "no_precondition"
    AnyDownstreamReady = "any_downstream_ready"
    AllDownstreamsReady = "all_downstreams_ready"


class DefaultSettings:
    time_unit: RedoxiTimeUnit = RedoxiTimeUnit.Microseconds
    step_interval: int = 2500
    wait_time_between_retry: int = 2500
    wait_time_retry_response: int = 100000
    number_of_retry: int = 3
    delivery_precondition: DeliveryPrecondition = DeliveryPrecondition.DontCare
    drop_strategy: DropStrategy = DropStrategy.DropAsNeeded
