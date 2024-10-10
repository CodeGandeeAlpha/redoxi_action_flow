from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = "src/flow_ros2_pipeline/config/config.yaml"
    return LaunchDescription(
        [
            Node(
                package="video_reader",
                namespace="",
                executable="video_reader_node",
                name="video_reader",
                parameters=[params_file],
            ),
            Node(
                package="master_node",
                namespace="",
                executable="master_node_node",
                name="master_node",
                parameters=[params_file],
            ),
            # Node(
            #     package="detector",
            #     namespace="",
            #     executable="pipeline_in_node",
            #     name="detector_in_node",
            #     parameters=[params_file],
            # ),
            Node(
                package="detector",
                namespace="",
                executable="pipeline_node",
                name="detector_pipeline_node",
                parameters=[params_file],
            ),
            Node(
                package="detector",
                namespace="",
                executable="ddq_detector_node.py",
                name="ddq_detector_node",
                parameters=[params_file],
            ),
            # Node(
            #     package="detector",
            #     namespace="",
            #     executable="pipeline_out_node",
            #     name="detector_out_node",
            #     parameters=[params_file],
            # ),
            Node(
                package="person_generator",
                namespace="",
                executable="person_generator_node",
                name="person_generator_node",
                parameters=[params_file],
            ),
        ]
    )
