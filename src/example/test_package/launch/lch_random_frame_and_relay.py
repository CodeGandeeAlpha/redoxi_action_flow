from launch import LaunchDescription
from launch_ros.actions import Node
import json


def generate_launch_description():
    video_sink_ns = "video_sink"

    json_params = {
        "declare_params": {
            "frame_interval_ms": 50000.0,
            "step_interval_ms": 10.0,
        },
        "downstreams": {
            "video_sink": {
                "accept_frame_actions": [f"/{video_sink_ns}/in/action"],
            },
        },
    }

    return LaunchDescription(
        [
            Node(
                package="test_package",
                executable="test_video_reader_random",
                name="video_source",
                namespace="video_source",
                parameters=[
                    {
                        "param_as_json_string": json.dumps(
                            json_params, separators=(",", ":")
                        ),
                    },
                ],
            ),
            Node(
                package="test_package",
                executable="test_video_sink",
                name="video_sink",
                namespace=video_sink_ns,
            ),
        ]
    )
