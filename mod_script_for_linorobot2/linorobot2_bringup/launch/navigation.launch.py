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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

# This function will be called by OpaqueFunction during launch
def _read_map_filename_and_set_config(context):
    # Reads the map_filename.conf file and returns a SetLaunchConfiguration action
    # with its content, or a default if an error occurs.
    # Resolve the path to map_filename.conf using the current launch context
    map_filename_conf_path_sub = PathJoinSubstitution([
        FindPackageShare('linorobot2_navigation'),
        'config',
        'map_filename.conf'
    ])
    # Perform the substitution to get the actual string path
    map_filename_conf_filepath = map_filename_conf_path_sub.perform(context)
    map_filename_to_set = 'map' # Default value in case of error
    try:
        with open(map_filename_conf_filepath, 'r') as f:
            map_filename_content = f.read().strip() # Read and strip whitespace
        
        map_filename_to_set = map_filename_content
        print(f"[MAP NAME READER] Successfully read map name from {map_filename_conf_filepath}: '{map_filename_to_set}'")
    except Exception as e:
        print(f"[MAP NAME READER] [ERROR] Error reading map_filename.conf: {e}. Using default '{map_filename_to_set}'.")
    
    # Return a SetLaunchConfiguration action to set the config
    # This action will be executed by the launch system.
    return [SetLaunchConfiguration('map_filename_from_file', map_filename_to_set)]

def generate_launch_description():
    depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_navigation.rviz']
    )

    # OpaqueFunction to read the map_filename.conf and set the 'map_filename_from_file' configuration
    # This must be placed before any actions that depend on 'map_filename_from_file'
    read_map_filename_action = OpaqueFunction(
        function=_read_map_filename_and_set_config,
        args=[],
        kwargs={}
    )

    # Map name file from path
    default_map_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'maps', LaunchConfiguration('map_filename_from_file')]
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'navigation.yaml']
    )

    nav2_sim_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'navigation_sim.yaml']
    )


    return LaunchDescription([
        # Execute the read map name function first
        read_map_filename_action,

        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=UnlessCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=IfCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_sim_config_path
            }.items()
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