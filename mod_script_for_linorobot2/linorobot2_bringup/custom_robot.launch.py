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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    # Define the path to your bridge script
    minibot_bridge_script_path = "/home/osboxes/linorobot2_ws/src/separate_structure/base_control_vdcs_bridge.py"

    return LaunchDescription([
        # #run robot driver
        # #https://github.com/linorobot/sphero_rvr
        # Node(
        #     package='sphero_rvr',
        #     executable='sphero_node',
        #     name='sphero_node',
        #     output='screen'
        # ),
        # #https://github.com/christianrauch/raspicam2_node
        # Node(
        #     package='raspicam2',
        #     executable='raspicam2_node',
        #     name='raspicam2',
        #     output='screen'
        # ),

        # Launch the bridge first, if you not need the bridge, just comment it out
        # This ExecuteProcess action will run your Python script.
        # It's placed at the beginning of the LaunchDescription list
        # to ensure it starts before other nodes.
        ExecuteProcess(
            cmd=['python3', minibot_bridge_script_path],
            output='screen', # This will print the script's output to the console
            name='minibot_base_bridge_process', # A unique name for this process
            # Optional: Set respawn=True if you want the bridge to automatically restart if it crashes
            # respawn=True,
            # respawn_delay=2.0, # Optional: Delay before respawning
        ),

        # My VDCS receiver
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/teensy',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='transport_udp', 
            default_value='false',
            description='Transport use UDP or UART mode, true to use UDP mode.'
        ),

        # Use "transport_udp" argument to ensure the transport mode use udp or uart
        Node(
            condition=IfCondition(LaunchConfiguration("transport_udp")),
            package='vdcs_receiver_node',
            executable='vdcs_receiver_node_udp',
            name='vdcs_receiver_node_udp',
            output='screen',
        ),

        Node(
            condition=UnlessCondition(LaunchConfiguration("transport_udp")),
            package='vdcs_receiver_node',
            executable='vdcs_receiver_node_ser',
            name='vdcs_receiver_node_serial',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('base_serial_port'),
            }]
        ),
        
        # Custom IMU Driver with/without magnetometer
        Node(
            package='serial_imu_bridge',
            executable='serial_imu_bridge',
            name='serial_imu_bridge',
            output='screen',
            parameters=[{
                # With magnetometer = /imu/data
                # Without magnetometer = /imu/data_raw
                'imu_topic': '/imu/data',
            }]
        ),

        #you can load your custom urdf launcher here
        #for demo's sake we'll use the default description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),
        #hardware/sensor specific launch files
        #for demo's sake we'll use the default description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        )
    ])