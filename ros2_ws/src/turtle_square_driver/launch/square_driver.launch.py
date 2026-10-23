from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the paths to your YAML parameter files
    waypoints_path = os.path.join(
        get_package_share_directory('turtle_square_driver'), 'config', 'waypoints.yaml'
    )
    stl_params_path = os.path.join(
        get_package_share_directory('turtle_square_driver'), 'config', 'stl_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtle_square_driver', 
            executable='square_driver',
            name='square_driver',
            output='screen',
            parameters=[waypoints_path]  # Pass waypoints to the driver
        ),
        Node(
            package='turtle_square_driver', 
            executable='manual_driver',
            name='manual_driver',
            output='screen',
            prefix=['gnome-terminal -- '] 
        ),
        Node(
            package='turtle_square_driver',
            executable='stl_processor',
            name='stl_processor',
            output='screen',
            parameters=[waypoints_path, stl_params_path]  # Pass both files to the processor
        ),
        Node(
            package='turtle_square_driver', 
            executable='stl_plotter.py',
            name='stl_plotter'
        )    
    ])