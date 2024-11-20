# Copyright 2023 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('gait_speed_ros2')
    
    # Get the launch directories for other packages
    # navigation_dir = get_package_share_directory('navigation_system')
    whisper_dir = get_package_share_directory('whisper_bringup')

    # HRI
    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(whisper_dir, 'launch', 'whisper.launch.py')
        )
    )
    audio_common_player_cmd = Node(
        package='audio_common',
        executable='audio_player_node',
        parameters=[
            {'channels': 2},
            {'device': -1}]
    )
    audio_common_tts_cmd = Node(
        package='tts_ros',
        executable='tts_node',
        parameters=[
            {'chunk': 4096},
            {'frame_id': ''},
            {'model': 'tts_models/en/ljspeech/vits'},
            {'speaker_wav': ''},
            {'device': 'cuda'}]
    )


    
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    print('params_file: ', params_file)
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['gait_speed_node']['ros__parameters']
    print(params)

    ld = LaunchDescription()


    robot_cmd = Node(
        package='gait_speed_ros2',
        executable='gait_speed_dist',
        output='screen',
        remappings=[
        ],
        parameters=[{
            'use_sim_time': True,
        }, params]
    )
    ld.add_action(whisper_cmd)
    ld.add_action(audio_common_player_cmd)
    ld.add_action(audio_common_tts_cmd)
    ld.add_action(robot_cmd)

    return ld

