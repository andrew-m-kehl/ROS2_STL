import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    eval_config_dir = os.path.join(
    get_package_share_directory('ros2_stl_eval'), 
    'config'
    )
    
    yaml_params = os.path.join(eval_config_dir, 'stl_params.yaml')
    stl_file_path = os.path.join(eval_config_dir, 'formulas.stl')
    
    return LaunchDescription([
        
        Node(
            package='ros2_stl_eval',
            executable='turtlesim_adapter',
            name='turtlesim_adapter'
        ),
        Node(
            package='ros2_stl_eval',
            executable='stl_processor',
            name='stl_processor',
            parameters=[yaml_params, {'formula_file': stl_file_path}],
            output='screen'
        ),

        Node(
            package='ros2_stl_eval',
            executable='stl_monitor',
            name='in_bounds',
            parameters=[yaml_params, {'formula_name': 'in_bounds'}],
            remappings=[('/stl_robustness', '/stl_metrics/in_bounds')],
            output='screen',
            prefix=['gnome-terminal -- '] 
        ),

        Node(
            package='ros2_stl_eval_plotter',
            executable='stl_plotter',
            name='stl_plotter'
        )
    ])