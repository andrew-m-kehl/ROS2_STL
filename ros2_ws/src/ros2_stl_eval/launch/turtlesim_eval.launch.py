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
            executable='generic_stl_monitor',
            name='stl_monitor_node',
            parameters=[yaml_params, {'formula_file': stl_file_path}],
            output='screen'
        ),

        Node(
            package='ros2_stl_eval',
            executable='event_listener',
            name='event_listener',
            parameters=[yaml_params, {
                'metrics_topic': '/stl_metrics',
                'verdicts_topic': '/stl_verdicts'
            }],
            output='screen'
        ),

        Node(
            package='ros2_stl_eval_plotter',
            executable='stl_plotter',
            name='stl_plotter',
            parameters=[yaml_params, {'metrics_topic': '/stl_verdicts'}],
            remappings=[('/stl_metrics', '/stl_verdicts')],
            output='screen'
        ),
    ])