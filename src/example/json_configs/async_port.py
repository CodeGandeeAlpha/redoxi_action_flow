source_node_json_params = {
    "timeunit": "ms",  # default time unit for all parameters, unless otherwise specified
    "declare_params": {
        "custom_var_1": 100.0,  # custom parameter example
        "custom_var_2": 10.0,  # custom parameter example
    },
    "runtime_config": {
        "frame_interval_ms": 2,  # The frame interval in ms, 0 means as fast as possible
        "step_interval_ms": 1,  # The step interval in ms
        "publish_to_debug_topic": True,  # Whether to publish to debug topic
        "delivery_policy_fallback": {
            "number_of_enqueue_retry": 0,  # Number of times to retry enqueueing
            "wait_time_between_enqueue_retry": 5.0,  # Wait time between enqueue retries in ms
            "number_of_delivery_retry": 10,  # Number of times to retry delivery
            "wait_time_between_delivery_retry": 10.0,  # Wait time between delivery retries in ms
            "wait_time_for_delivery_response": 100.0,  # Wait time for delivery response in ms
        },
        "delivery_options": {
            "frame_payload_type": "uncompressed",  # can be "uncompressed", "uncompressed_by_shared_memory", "jpeg_encoded", "png_encoded"
            "drop_frame_strategy": "drop_as_needed",  # can be "no_drop" or "drop_as_needed"
            "jpeg_quality": 90,  # only valid when frame_payload_type is "jpeg_encoded"
            "num_buffer_frames": 1,  # number of frames to buffer waiting for delivery
        },
    },
    "init_config": {
        "use_debug_pub": True,  # Whether to create debug publisher
        "downstreams": {
            "actions": [
                {
                    "name": "/video_sink/in/action",  # Name of the downstream action
                    "delivery_policy": {
                        "number_of_enqueue_retry": 10,  # Number of times to retry enqueueing
                        "wait_time_between_enqueue_retry": 5.0,  # Wait time between enqueue retries in ms
                        "number_of_delivery_retry": 10,  # Number of times to retry delivery
                        "wait_time_between_delivery_retry": 10.0,  # Wait time between delivery retries in ms
                        "wait_time_for_delivery_response": 50.0,  # Wait time for delivery response in ms
                    },
                }
            ]
        },
    },
}
