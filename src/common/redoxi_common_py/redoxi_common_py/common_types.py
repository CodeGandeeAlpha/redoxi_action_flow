from enum import Enum

__all__ = ["RedoxiTimeUnit", "DropStrategy", "DeliveryPrecondition"]


# default to microseconds
class RedoxiTimeUnit(str, Enum):
    Hours = "hours"
    Minutes = "minutes"
    Seconds = "seconds"
    Milliseconds = "ms(1e-3)"
    Microseconds = "us(1e-6)"
    Nanoseconds = "ns(1e-9)"
    Default = Microseconds


class DropStrategy(str, Enum):
    DontCare = "dont_care"
    NoDrop = "no_drop"
    DropAsNeeded = "drop_as_needed"


class DeliveryPrecondition(str, Enum):
    DontCare = "dont_care"
    NoPrecondition = "no_precondition"
    AnyDownstreamReady = "any_downstream_ready"
    AllDownstreamsReady = "all_downstreams_ready"
