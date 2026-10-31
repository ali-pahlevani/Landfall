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

"""Landfall's map loader produces exactly the grid nav2_map_server publishes for the same file."""

import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS = {
    'depot': os.path.join(TEST_DIR, 'fixtures', 'depot_small', 'depot.yaml'),
    'trinary_color': os.path.join(TEST_DIR, 'maps', 'trinary_color.yaml'),
    'scale_alpha': os.path.join(TEST_DIR, 'maps', 'scale_alpha.yaml'),
    'raw_negate': os.path.join(TEST_DIR, 'maps', 'raw_negate.yaml'),
}


@pytest.mark.launch_test
def generate_test_description():
    servers = []
    for name, yaml in MAPS.items():
        servers.append(launch_ros.actions.Node(
            package='nav2_map_server', executable='map_server', name='map_server_' + name,
            parameters=[{'yaml_filename': yaml, 'topic_name': 'parity/' + name}]))
    manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_parity',
        parameters=[{'autostart': True, 'node_names': ['map_server_' + n for n in MAPS]}])
    arguments = []
    for name, yaml in MAPS.items():
        arguments += ['/parity/' + name, yaml]
    checker = launch.actions.ExecuteProcess(
        cmd=[os.environ['LANDFALL_MAP_PARITY_CHECK']] + arguments, output='screen')
    return launch.LaunchDescription(
        servers + [manager, checker, launch_testing.actions.ReadyToTest()]), {'checker': checker}


class TestChecker(unittest.TestCase):

    def test_checker_finishes(self, proc_info, checker):
        proc_info.assertWaitForShutdown(process=checker, timeout=120)


@launch_testing.post_shutdown_test()
class TestParity(unittest.TestCase):

    def test_every_grid_matches(self, proc_info, checker):
        launch_testing.asserts.assertExitCodes(proc_info, process=checker)
