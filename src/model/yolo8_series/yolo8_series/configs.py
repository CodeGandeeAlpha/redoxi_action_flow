import redoxi_common_py.configs.async_port_configs as pc

try:
    from attrs import define, field
except ImportError:
    from attr import define, field


@define(kw_only=True)
class ImageRequestConfig:
    _time_unit: pc.RedoxiTimeUnit = field(factory=pc.RedoxiTimeUnit.Default)
    input_port_config: pc.InputPortConfig = field(factory=pc.InputPortConfig)
    output_port_config: pc.OutputPortConfig = field(factory=pc.OutputPortConfig)
    output_enqueue_policy: pc.DeliveryPolicy = field(factory=pc.DeliveryPolicy)
    publish_visualization_topic: str | None = field(factory=None)
    publish_probe_detection_done_topic: str | None = field(factory=None)


@define(kw_only=True)
class InitConfig:
    model_configs: list[pc.ModelConfig] = field(factory=list)

    # input port that receives detection request, this is the default port
    detection_request_config: pc.InputPortConfig | None = field(
        factory=pc.InputPortConfig
    )

    # additionally, input port that receives image request, and the model node sends out
    # request response to downstreams
    image_request_config: pc.InputPortConfig | None = field(factory=None)
