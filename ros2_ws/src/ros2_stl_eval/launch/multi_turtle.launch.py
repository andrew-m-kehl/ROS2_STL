import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    driver_waypoints = os.path.join(
        get_package_share_directory('driver'), 'config', 'waypoints.yaml'
    )
    eval_config_dir = os.path.join(
        get_package_share_directory('ros2_stl_eval'), 'config'
    )
    yaml_params = os.path.join(eval_config_dir, 'stl_params.yaml')
    stl_file_path = os.path.join(eval_config_dir, 'formulas.stl')

    # Spawn turtle2 at (2.5, 2.5) after a 1-second delay so turtlesim_node is ready
    spawn_turtle2 = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                    "{x: 2.5, y: 2.5, theta: 0.0, name: 'turtle2'}"
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # --- Simulator ---
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        spawn_turtle2,

        # ==========================================
        # PIPELINE 1: turtle1
        # ==========================================
        Node(
            package='driver',
            executable='point_driver',
            name='point_driver_1',
            parameters=[driver_waypoints, {'max_speed': 2.0}],
            output='screen'
        ),
        Node(
            package='ros2_stl_eval',
            executable='turtlesim_adapter',
            name='adapter_1',
            remappings=[('/stl_input', '/turtle1/stl_input')]
        ),
        Node(
            package='ros2_stl_eval',
            executable='stl_processor',
            name='stl_processor_1',
            parameters=[
                yaml_params,
                {
                    'formula_file': stl_file_path,
                    'input_topic': '/turtle1/stl_input',
                    'metrics_topic': '/turtle1/stl_metrics'
                }
            ],
            output='screen'
        ),
        Node(
            package='ros2_stl_eval_plotter',
            executable='stl_plotter',
            name='stl_plotter_1',
            parameters=[yaml_params, {'metrics_topic': '/turtle1/stl_metrics'}],
            remappings=[('/stl_metrics', '/turtle1/stl_metrics')],
            output='screen'
        ),

        # ==========================================
        # PIPELINE 2: turtle2
        # ==========================================
        Node(
            package='driver',
            executable='point_driver',
            name='point_driver_2',
            parameters=[driver_waypoints, {'max_speed': 6.0}],
            remappings=[
                ('/turtle1/pose', '/turtle2/pose'),
                ('/turtle1/cmd_vel', '/turtle2/cmd_vel')
            ],
            output='screen'
        ),
        Node(
            package='ros2_stl_eval',
            executable='turtlesim_adapter',
            name='adapter_2',
            remappings=[
                ('/turtle1/pose', '/turtle2/pose'),
                ('/stl_input', '/turtle2/stl_input')
            ]
        ),
        Node(
            package='ros2_stl_eval',
            executable='stl_processor',
            name='stl_processor_2',
            parameters=[
                yaml_params,
                {
                    'formula_file': stl_file_path,
                    'input_topic': '/turtle2/stl_input',
                    'metrics_topic': '/turtle2/stl_metrics'
                }
            ],
            output='screen'
        ),
        Node(
            package='ros2_stl_eval_plotter',
            executable='stl_plotter',
            name='stl_plotter_2',
            parameters=[yaml_params, {'metrics_topic': '/turtle2/stl_metrics'}],
            remappings=[('/stl_metrics', '/turtle2/stl_metrics')],
            output='screen'
        ),
    ])