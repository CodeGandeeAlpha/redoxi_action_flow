from attrs import define, field, asdict
import attrs.validators as av
from typing import Literal
from redoxi_common_py.common_types import *

__all__ = [
    "JsonConvertible",
    "RedoxiTimeUnit",
    "DropStrategy",
    "DeliveryPrecondition",
    "ModelConfig",
    "InputPortConfig",
    "RetryPolicy",
    "DeliveryPolicy",
    "DownstreamSpec",
    "OutputPortConfig",
]


@define(kw_only=True)
class InputPortConfig(JsonConvertible):
    _action_goal_type: str | None = field(default=None)
    _time_unit: RedoxiTimeUnit = field(default=DefaultSettings.time_unit)
    buffer_capacity: int | None = field(default=None)
    action_name: str = field()
    goal_result_expire_time: int | None = field(default=None)


@define(kw_only=True)
class RetryPolicy(JsonConvertible):
    _time_unit: RedoxiTimeUnit = field(default=DefaultSettings.time_unit)
    number_of_retry: int | None = field(default=DefaultSettings.number_of_retry)
    wait_time_between_retry: int | None = field(
        default=DefaultSettings.wait_time_between_retry
    )
    wait_time_retry_response: int | None = field(
        default=DefaultSettings.wait_time_retry_response
    )
    fallback_number_of_retry: int | None = field(default=None)
    fallback_wait_time_between_retry: int | None = field(default=None)
    fallback_wait_time_retry_response: int | None = field(default=None)


@define(kw_only=True)
class DeliveryPolicy(JsonConvertible):
    retry_policy: RetryPolicy = field(factory=RetryPolicy)
    precondition: DeliveryPrecondition = field(
        default=DefaultSettings.delivery_precondition
    )
    drop_strategy: DropStrategy = field(default=DefaultSettings.drop_strategy)


@define(kw_only=True)
class DownstreamSpec(JsonConvertible):
    name: str = field()
    action_name: str = field()
    delivery_policy: DeliveryPolicy = field(factory=DeliveryPolicy)
    create_debug_pub: bool = field(default=False)
    debug_topic_source_data_sending: str | None = field(default=None)
    debug_topic_source_data_succeeded: str | None = field(default=None)
    debug_topic_source_data_failed: str | None = field(default=None)
    debug_topic_target_data_sending: str | None = field(default=None)
    debug_topic_target_data_succeeded: str | None = field(default=None)
    debug_topic_target_data_failed: str | None = field(default=None)


@define(kw_only=True)
class OutputPortConfig(JsonConvertible):
    _action_goal_type: str | None = field(default=None)
    downstream_specs: list[DownstreamSpec] = field(factory=list)
    num_buffer_requests: int = field(default=1)
    preserve_request_order: bool = field(default=True)
    fallback_delivery_precondition: DeliveryPrecondition = field(
        default=DefaultSettings.delivery_precondition
    )
