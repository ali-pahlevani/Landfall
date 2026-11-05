# Copyright 2026 Ali Pahlevani
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

"""AMCL, started without a pose, converges after the relocalizer publishes /initialpose."""

# map_server, AMCL and the relocalizer run under nav2_lifecycle_manager (with bonds), as in a
# robot's localization bringup. relocalization_check plays the depot fixture's first window and
# decides.

import os
import tempfile
import unittest

import launch
import launch.actions
import launch.event_handlers
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

FIXTURE = os.environ['LANDFALL_FIXTURE_DIR']
WORK = tempfile.mkdtemp(prefix='landfall_amcl_test_')
DATABASE = os.path.join(WORK, 'depot.landfall.db')
AMCL_PARAMS = os.environ['LANDFALL_AMCL_PARAMS']


@pytest.mark.launch_test
def generate_test_description():
    build = launch.actions.ExecuteProcess(
        cmd=[os.environ['LANDFALL_CLI'], 'build',
             '--map', os.path.join(FIXTURE, 'depot.yaml'),
             '--bag', os.path.join(FIXTURE, 'bag'),
             '--config', os.path.join(FIXTURE, 'build.yaml'),
             '--output', DATABASE, '--threads', '4'],
        output='screen')
    sim_time = {'use_sim_time': True}
    map_server = launch_ros.actions.Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[sim_time, {'yaml_filename': os.path.join(FIXTURE, 'depot.yaml')}])
    amcl = launch_ros.actions.Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[AMCL_PARAMS, sim_time])
    relocalizer = launch.actions.ExecuteProcess(
        cmd=[os.environ['LANDFALL_RELOCALIZER'], '--ros-args',
             '-p', 'use_sim_time:=true',
             '-p', 'database_path:=' + DATABASE,
             '-p', 'image.topic:=/rgbd_camera/image',
             '-p', 'image.transport:=compressed',
             '-p', 'camera_info.topic:=/rgbd_camera/camera_info',
             '-p', 'imu.topic:=/imu',
             '-p', 'imu.frame:=imu_link',
             '-p', 'consistency.num_frames:=2'],
        output='screen')
    manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 10.0,
                     'node_names': ['map_server', 'amcl', 'landfall_relocalizer']}])
    checker = launch.actions.ExecuteProcess(
        cmd=[os.environ['LANDFALL_RELOCALIZATION_CHECK'], os.path.join(FIXTURE, 'bag'),
             '126.0', '161.9'],
        output='screen')
    stack = launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(
        target_action=build, on_exit=[map_server, amcl, relocalizer, manager, checker]))
    return launch.LaunchDescription([build, stack, launch_testing.actions.ReadyToTest()]), {
        'build': build, 'checker': checker}


class TestRelocalization(unittest.TestCase):

    def test_checker_finishes(self, proc_info, checker):
        proc_info.assertWaitForShutdown(process=checker, timeout=240)


@launch_testing.post_shutdown_test()
class TestAmclConverged(unittest.TestCase):

    def test_database_built_and_amcl_converged(self, proc_info, build, checker):
        launch_testing.asserts.assertExitCodes(proc_info, process=build)
        launch_testing.asserts.assertExitCodes(proc_info, process=checker)
