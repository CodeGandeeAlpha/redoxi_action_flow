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
