# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Launch a local talker plus a remote talker and a listener."""
import launch_ros
from launch import LaunchDescription, LaunchIntrospector
from swri_launch_test import SshMachine


def generate_launch_description():
    remote_machine = SshMachine(hostname="grandgrowl.local", env='. /opt/ros/dashing/setup.bash')
    ld = LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='talker', output='screen',
            remappings=[('chatter', 'my_chatter')],
            machine=remote_machine,
        ),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='talker', output='screen',
            remappings=[('chatter', 'my_chatter')],
        ),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='listener', output='screen',
            remappings=[('chatter', 'my_chatter')],
            machine=remote_machine
        ),
    ])

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    return ld
