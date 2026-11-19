import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('ros2_stl_eval'),
        'config',
        'stl_params.yaml'
    )

    return LaunchDescription([
        
        Node(
            package='ros2_stl_eval',
            executable='odom_adapter',
            name='turtlesim_adapter'
        ),
        Node(
            package='ros2_stl_eval',
            executable='stl_processor',
            name='stl_processor',
            parameters=[params_file]
        ),
        Node(
            package='ros2_stl_eval_plotter',
            executable='stl_plotter',
            name='stl_plotter'
        )
    ])