from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('robot_description')

    params = os.path.join(pkg, 'config', 'navigation_params.yaml')
    docksParams = os.path.join(pkg, 'config', 'docks.yaml')
    map_file = os.path.expanduser('~/my_map2.yaml')

    return LaunchDescription([

        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            output='screen',
            parameters=[params]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[params]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[params]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[params]
        ),

        Node(
            package='opennav_docking',
            executable='opennav_docking',
            output='screen',
            parameters=[
                params,
                {'dock_database': docksParams},
                {'use_sim_time': True}
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='home_dock_tf',
            arguments=['1.0', '0.0', '0.0', '0', '0', '0', 'map', 'home_dock_frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='work_dock_tf',
            arguments=['3.5', '-1.2', '0.0', '0', '0', '1.57', 'map', 'work_dock_frame']
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server'
                ]
            }]
        ),
    ])
