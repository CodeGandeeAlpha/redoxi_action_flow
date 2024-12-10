import redoxi_common_py.configs.async_port_configs as portConfig
import redoxi_common_py.configs.node_configs as nodeConfig
from typing import Any, Literal

try:
    from attrs import define, field, asdict
except ImportError:
    from attr import define, field, asdict


@define(kw_only=True)
class ModelPostprocessConfig(portConfig.JsonConvertible):
    conf_threshold: float = field(default=0.25)
    iou_threshold: float = field(default=0.45)


@define(kw_only=True)
class ImageRequestConfig(portConfig.JsonConvertible):
    input_port_config: portConfig.InputPortConfig | None = field(default=None)
    output_port_config: portConfig.OutputPortConfig | None = field(default=None)
    output_enqueue_policy: portConfig.DeliveryPolicy | None = field(default=None)


@define(kw_only=True)
class InitConfig(nodeConfig.BaseRosNodeInitConfig):
    model_configs: list[nodeConfig.ModelConfig] = field(factory=list)
    detection_request_config: portConfig.InputPortConfig | None = field(default=None)
    image_request_config: ImageRequestConfig | None = field(default=None)
    publish_visualization_topic: str | None = field(default=None)
    publish_probe_detection_done_topic: str | None = field(default=None)


@define(kw_only=True)
class RuntimeConfig(nodeConfig.BaseRosNodeRuntimeConfig):
    enable_blocking_mode: bool = field(default=False)
    model_output_config: ModelPostprocessConfig = field(factory=ModelPostprocessConfig)
    enable_visualization: bool = field(default=True)
    enable_performance_probe: bool = field(default=False)


@define(kw_only=True)
class Yolo8ModelConfig(portConfig.JsonConvertible):
    declare_params: dict[str, Any] | None = field(default=None)
    init_config: InitConfig = field(factory=InitConfig)
    runtime_config: RuntimeConfig = field(factory=RuntimeConfig)
