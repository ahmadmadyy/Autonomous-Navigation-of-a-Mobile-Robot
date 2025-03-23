import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('project_localization'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('maps_server'), 'config', 'map_area.yaml')
    
    controller_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'recovery.yaml')

    # Localization Nodes (Start Immediately)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_file}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # Delay the following nodes by 5 seconds to allow localization to stabilize
    delayed_nodes = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            Node(
                package='project_localization',
                executable='spot_recorder',
                name='spot_recorder',
                output='screen'),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml]),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_yaml]),
                
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='recoveries_server',
                parameters=[recovery_yaml],
                output='screen'),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml]),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_pathplanner',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': ['planner_server',
                                            'controller_server',
                                            'recoveries_server',
                                            'bt_navigator']}])
        ]
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager_localization,
        delayed_nodes  # All following nodes will start after 5 seconds
    ])
