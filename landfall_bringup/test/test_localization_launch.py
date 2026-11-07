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

"""localization.launch.xml brings map_server, AMCL and the relocalizer up under one manager."""

# A small free map and no database: the relocalizer still activates and answers NOT_READY. This
# checks the launch file on every distro (arguments, node names, parameters, the lifecycle
# manager); the simulation runs check relocalization through it (docs/nav2_integration.md). The
# nodes run in a namespace: colcon tests packages in parallel, and landfall_ros's launch tests
# start nodes with these names.

import os
import tempfile
import unittest

from bringup_test_support import call, wait_until_active, write_free_map
from landfall_msgs.msg import RelocalizationResult
from landfall_msgs.srv import Relocalize
import launch
import launch.actions
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy

WORK = tempfile.mkdtemp(prefix='landfall_localization_launch_')
LAUNCH = os.path.join(os.environ['LANDFALL_BRINGUP_SHARE'], 'launch', 'localization.launch.xml')
NAMESPACE = 'landfall_localization_launch_test'


@pytest.mark.launch_test
def generate_test_description():
    map_yaml = write_free_map(WORK)
    params = os.path.join(WORK, 'nav2_params.yaml')
    with open(params, 'w') as f:
        f.write('/**/amcl:\n  ros__parameters:\n    scan_topic: scan\n'
                '/**/map_server:\n  ros__parameters:\n    topic_name: map\n')
    localization = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource(LAUNCH),
        launch_arguments={
            'map': map_yaml,
            'database': os.path.join(WORK, 'missing.landfall.db'),
            'params_file': params,
            'namespace': NAMESPACE,
        }.items())
    return launch.LaunchDescription([localization, launch_testing.actions.ReadyToTest()])


class TestLocalizationLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('localization_launch_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_nodes_active_and_relocalizer_answers(self):
        for name in ('map_server', 'amcl', 'landfall_relocalizer'):
            self.assertTrue(wait_until_active(self.node, f'{NAMESPACE}/{name}', 90.0), name)
        client = self.node.create_client(
            Relocalize, f'/{NAMESPACE}/landfall_relocalizer/relocalize')
        response = call(self.node, client, Relocalize.Request(), 20.0)
        self.assertIsNotNone(response)
        self.assertEqual(response.result.status, RelocalizationResult.NOT_READY,
                         response.result.message)
