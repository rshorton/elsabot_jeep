import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml

#KEEP_OUT='backyard'
KEEP_OUT='upstairs'

def generate_launch_description():

    elsabot_jeep_dir = get_package_share_directory('elsabot_jeep')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    ko_params_file = LaunchConfiguration('keep_out_params_file')
    ko_mask_yaml_file = LaunchConfiguration('keep_out_map')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_ko_params_file_cmd = DeclareLaunchArgument(
        'keep_out_params_file',
        default_value=os.path.join(elsabot_jeep_dir, 'config', 'keep_out.yaml'),
        description='Full path to the keep out parameters file to use')

    declare_ko_mask_yaml_file_cmd = DeclareLaunchArgument(
        'keep_out_map',
        default_value=os.path.join(elsabot_jeep_dir, 'maps', f'{KEEP_OUT}_keep_out.yaml'),
        description='Full path to filter mask yaml file to load')

    # Make re-written yaml for keep-out param file to update with map path
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': ko_mask_yaml_file}

    configured_params = RewrittenYaml(
        source_file=ko_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)


    # Create our own temporary YAML files that include substitutions
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_ko_params_file_cmd)
    ld.add_action(declare_ko_mask_yaml_file_cmd)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)

    return ld
