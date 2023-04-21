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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

MAP_NAME='upstairs'
#MAP_NAME='backyard'

def generate_launch_description():

    elsabot_jeep_dir = get_package_share_directory('elsabot_jeep')

    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'launch', 'sensors.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_jeep'), 'launch', 'description.launch.py']
    )

    rosbridge_launch_path = PathJoinSubstitution(
        [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("elsabot_jeep"), "config", "ekf.yaml"]
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

    return LaunchDescription([

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
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Run SLAM to create a map'
        ),

        DeclareLaunchArgument(
            'use_keep_out',
            default_value='True',
            description='Enable use of keep-out area map'
        ),

        DeclareLaunchArgument(
            'use_nav',
            default_value='False',
            description='Enable also start nav2'
        ),

        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/teensy',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='joy', 
            default_value='false',
            description='Use Joystick'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            condition=IfCondition(LaunchConfiguration("joy")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('elsabot_jeep'), 'launch', 'navigation.launch.py')),
            condition=IfCondition(LaunchConfiguration("use_nav")),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'map': LaunchConfiguration("map"),
                'slam': LaunchConfiguration("slam"),
                'use_keep_out': LaunchConfiguration("use_keep_out"),
                'keep_out_map': LaunchConfiguration("keep_out_map")}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
        ),

        # web bridge (for proxying topics/actions to/from ros_web based applications)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_path)
        )
    ])
    return ld
