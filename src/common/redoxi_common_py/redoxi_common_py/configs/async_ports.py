from attrs import define, field, asdict
import attrs.validators as av
from typing import Literal
import redoxi_common_py.common_types as commonTypes
from redoxi_common_py.common_types import *

__all__ = [
    "InputPortConfig",
    "RetryPolicy",
    "DeliveryPolicy",
    "DownstreamSpec",
    "OutputPortConfig",
] + commonTypes.__all__


@define(kw_only=True)
class InputPortConfig(commonTypes.JsonConvertible):
    _action_goal_type: str | None = field(default=None)
    _time_unit: commonTypes.RedoxiTimeUnit = field(
        default=commonTypes.DefaultSettings.time_unit
    )
    buffer_capacity: int | None = field(default=1)
    action_name: str = field()
    goal_result_expire_time: int | None = field(default=None)


@define(kw_only=True)
class RetryPolicy(commonTypes.JsonConvertible):
    _time_unit: commonTypes.RedoxiTimeUnit = field(
        default=commonTypes.DefaultSettings.time_unit
    )
    number_of_retry: int | None = field(
        default=commonTypes.DefaultSettings.number_of_retry
    )
    wait_time_between_retry: int | None = field(
        default=commonTypes.DefaultSettings.wait_time_between_retry
    )
    wait_time_retry_response: int | None = field(
        default=commonTypes.DefaultSettings.wait_time_retry_response
    )
    fallback_number_of_retry: int | None = field(default=None)
    fallback_wait_time_between_retry: int | None = field(default=None)
    fallback_wait_time_retry_response: int | None = field(default=None)


@define(kw_only=True)
class DeliveryPolicy(commonTypes.JsonConvertible):
    retry_policy: RetryPolicy = field(factory=RetryPolicy)
    precondition: commonTypes.DeliveryPrecondition | None = field(default=None)
    drop_strategy: commonTypes.DropStrategy | None = field(default=None)


@define(kw_only=True)
class DownstreamSpec(commonTypes.JsonConvertible):
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
class OutputPortConfig(commonTypes.JsonConvertible):
    _action_goal_type: str | None = field(default=None)
    downstream_specs: list[DownstreamSpec] = field(factory=list)
    num_buffer_requests: int = field(default=1)
    preserve_request_order: bool = field(default=True)
    fallback_delivery_precondition: commonTypes.DeliveryPrecondition = field(
        default=commonTypes.DefaultSettings.delivery_precondition
    )
    data_topic_for_source_data: str | None = field(default=None)
    data_topic_for_target_data: str | None = field(default=None)
    visualization_topic_for_source_data: str | None = field(default=None)
    visualization_topic_for_target_data: str | None = field(default=None)
    probe_topic_for_source_data: str | None = field(default=None)
    probe_topic_for_target_data: str | None = field(default=None)
