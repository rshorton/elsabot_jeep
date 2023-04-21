# Portions of this file are from the Elsabot project by Scott Horton
# and others from LinoRobot2

# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

MAP_NAME='upstairs'
#MAP_NAME='backyard'

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'rviz', 'navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'maps', f'{MAP_NAME}.yaml']
    )

    default_keep_out_map_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'maps', f'{MAP_NAME}_keep_out.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'config', 'navigation.yaml']
#        [FindPackageShare('elsabot_jeep'), 'config', 'navigation_depth.yaml']
    )

    nav2_bt_through_poses_xml_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'config', 'navigate_through_poses_w_replanning_and_recovery.xml']
    )

    nav2_bt_to_pose_xml_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'config', 'navigate_to_pose_w_replanning_and_recovery.xml']
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'default_nav_through_poses_bt_xml': LaunchConfiguration("default_nav2_bt_through_poses_xml_path"),
        'default_nav_to_pose_bt_xml': LaunchConfiguration("default_nav2_bt_to_pose_xml_path")}

    # Re-write the nav parameters file to use our modified nav bt xml files.
    # Those files remove spin recovery since that is not possible on the jeep.
    rewritten_nav2_params = RewrittenYaml(
            source_file=nav2_config_path,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Run SLAM to create a map'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='keep_out_map', 
            default_value=default_keep_out_map_path,
            description='Navigation keep out map path'
        ),

        DeclareLaunchArgument(
            'use_keep_out',
            default_value='True',
            description='Enable use of keep-out area map'
        ),

        DeclareLaunchArgument(
            name='default_nav2_bt_through_poses_xml_path',
            default_value=nav2_bt_through_poses_xml_path,
            description='Full path to Nav behavior tree xml file'),

        DeclareLaunchArgument(
            name='default_nav2_bt_to_pose_xml_path',
            default_value=nav2_bt_to_pose_xml_path,
            description='Full path to Nav behavior tree xml file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'map': LaunchConfiguration("map"),
                'slam': LaunchConfiguration("slam"),
                'log_level': "info",
                'params_file': rewritten_nav2_params
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_jeep'), 'launch', 'keep_out_area.launch.py')),
            condition=IfCondition(LaunchConfiguration("use_keep_out")),
            launch_arguments={
                'keep_out_map': LaunchConfiguration("keep_out_map"),
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'autostart': LaunchConfiguration("autostart")}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])
