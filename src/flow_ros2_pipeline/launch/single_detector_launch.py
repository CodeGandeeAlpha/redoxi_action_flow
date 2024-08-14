from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = 'src/flow_ros2_pipeline/config/config.yaml'
    return LaunchDescription([
        Node(
            package='detector',
            namespace='',
            executable='ddq_detector_node.py',
            name='ddq_detector_node',
            parameters=[params_file]
        )
    ])