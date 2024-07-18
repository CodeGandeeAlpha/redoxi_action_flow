from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = 'src/flow_ros2_pipeline/config/config.yaml'
    return LaunchDescription([
        Node(
            package='video_reader',
            namespace='',
            executable='video_reader_node',
            name='video_reader',
            parameters=[params_file]
        ),
        Node(
            package='master_node',
            namespace='',
            executable='master_node_node',
            name='master_node',
            parameters=[params_file]
        )
    ])