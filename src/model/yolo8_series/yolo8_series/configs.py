from redoxi_common_py.configs.base_node import *
import redoxi_common_py.configs.base_node as baseNodeCfg

from redoxi_common_py.configs.async_ports import *
import redoxi_common_py.configs.async_ports as portCfg
from typing import Any, Literal

try:
    from attrs import define, field, asdict
except ImportError:
    from attr import define, field, asdict

ExampleDetectionModelConfig = {
    "declare_params": {},
    "init_config": {
        "model_configs": [
            {
                "model_path": "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8s.onnx",
                "device_type": "cpu",
                "device_index": 0,
            }
        ],
        "detection_request_config": {
            "input_port_config": {
                "_action_goal_type": "redoxi_public_msgs/action/ProcessDetectionsByFrame_Goal",
                "buffer_capacity": -1,
                "action_name": "in/detection_request",
                "goal_result_expire_time": 1000000,
            }
        },
        "image_request_config": {
            "input_port_config": {
                "_action_goal_type": "redoxi_public_msgs/action/ProcessFrame_Goal",
                "buffer_capacity": -1,
                "action_name": "in/image_request",
                "goal_result_expire_time": 1000000,
            },
            "output_port_config": {
                "_action_goal_type": "redoxi_public_msgs/action/ProcessDetections_Goal",
                "downstream_specs": [
                    {
                        "name": "",
                        "action_name": "/detection_sink/in/detection_response",
                        "delivery_policy": {
                            "retry_policy": {
                                "fallback_number_of_retry": 3,
                                "fallback_wait_time_between_retry": 5000,
                                "fallback_wait_time_retry_response": 100000,
                            },
                            "precondition": "dont_care",
                            "drop_strategy": "dont_care",
                        },
                        "create_debug_pub": False,
                    }
                ],
                "num_buffer_requests": 1,
                "preserve_request_order": True,
                "fallback_delivery_precondition": "dont_care",
            },
            "output_enqueue_policy": {
                "retry_policy": {
                    "fallback_number_of_retry": 3,
                    "fallback_wait_time_between_retry": 5000,
                    "fallback_wait_time_retry_response": 100000,
                },
                "precondition": "dont_care",
                "drop_strategy": "dont_care",
            },
        },
        "publish_visualization_topic": "debug/visualization",
        "publish_probe_detection_done_topic": "probe/detection_done",
        "_time_unit": "us(1e-6)",
    },
    "runtime_config": {
        "enable_blocking_mode": False,
        "model_output_config": {"conf_threshold": 0.25, "iou_threshold": 0.45},
        "enable_visualization": True,
        "enable_performance_probe": False,
        "_time_unit": "us(1e-6)",
        "step_interval": 5000,
    },
}

__all__ = (
    [
        "ModelPostprocessConfig",
        "ImageRequestConfig",
        "Yolo8ModelInitConfig",
        "Yolo8ModelRuntimeConfig",
        "Yolo8ModelNodeConfig",
        "DetectionRequestConfig",
    ]
    + baseNodeCfg.__all__
    + portCfg.__all__
)


@define(kw_only=True)
class ModelPostprocessConfig(JsonConvertible):
    conf_threshold: float = field(default=0.25)
    iou_threshold: float = field(default=0.45)
    selected_class_ids: list[int] = field(factory=list)


@define(kw_only=True)
class ImageRequestConfig(JsonConvertible):
    """
    ImageRequestConfig is used to configure the image request configuration.

    input_port_config:
        action type:  redoxi_public_msgs/action/ProcessFrame_Goal
    output_port_config:
        action type:  redoxi_public_msgs/action/ProcessDetections_Goal
    """

    input_port_config: InputPortConfig | None = field(default=None)
    output_port_config: OutputPortConfig | None = field(default=None)
    output_enqueue_policy: DeliveryPolicy | None = field(default=None)


@define(kw_only=True)
class DetectionRequestConfig(JsonConvertible):
    input_port_config: InputPortConfig | None = field(default=None)


@define(kw_only=True)
class Yolo8ModelInitConfig(BaseRosNodeInitConfig):
    """
    Yolo8ModelInitConfig is used to configure the YOLOv8 model initialization.

    detection request port:
        action type:  redoxi_public_msgs/action/ProcessDetectionsByFrame_Goal
    image request port:
        input_port_config:
            action type:  redoxi_public_msgs/action/ProcessFrame_Goal
        output_port_config:
            action type:  redoxi_public_msgs/action/ProcessDetections_Goal
    """

    model_configs: list[ModelConfig] = field(factory=list)
    detection_request_config: DetectionRequestConfig | None = field(default=None)
    image_request_config: ImageRequestConfig | None = field(default=None)
    publish_visualization_topic: str | None = field(default=None)
    publish_probe_detection_done_topic: str | None = field(default=None)


@define(kw_only=True)
class Yolo8ModelRuntimeConfig(BaseRosNodeRuntimeConfig):
    enable_blocking_mode: bool = field(default=False)
    model_output_config: ModelPostprocessConfig = field(factory=ModelPostprocessConfig)
    enable_visualization: bool = field(default=False)
    enable_performance_probe: bool = field(default=False)


@define(kw_only=True)
class Yolo8ModelNodeConfig(BaseRosNodeConfig):
    init_config: Yolo8ModelInitConfig = field(factory=Yolo8ModelInitConfig)
    runtime_config: Yolo8ModelRuntimeConfig = field(factory=Yolo8ModelRuntimeConfig)
