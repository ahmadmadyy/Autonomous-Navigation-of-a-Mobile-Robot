import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('project_localization'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('maps_server'), 'config', 'map_area.yaml')
    #rviz_dir = os.path.join(get_package_share_directory('project_localization'), 'rviz')
    #rviz_file = os.path.join(rviz_dir, 'pathplanning_rviz_config.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        Node(
            package='project_localization',
            executable='spot_recorder',
            name='spot_recorder',
            output='screen'),
    ])