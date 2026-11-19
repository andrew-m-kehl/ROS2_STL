import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Pull the waypoints from the new centralized eval package
    params_file = os.path.join(
        get_package_share_directory('driver'),
        'config',
        'waypoints.yaml'
    )

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # Node(
        #     package='driver', 
        #     executable='manual_driver',
        #     name='manual_driver',
        #     output='screen',
        #     prefix=['gnome-terminal -- '] 
        # ),

        Node(
            package='driver',
            executable='point_driver',
            name='point_driver',
            parameters=[params_file]
        ),
    ])


