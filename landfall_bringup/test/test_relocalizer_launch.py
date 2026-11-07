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

"""relocalizer.launch.xml starts the relocalizer with a lifecycle manager of its own."""

# No database and no AMCL: the relocalizer still activates and answers NOT_READY, which checks the
# launch file on every distro (docs/nav2_integration.md). In a namespace: colcon tests packages in
# parallel, and landfall_ros's tests start a landfall_relocalizer too.

import os
import tempfile
import unittest

from bringup_test_support import call, wait_until_active
from landfall_msgs.msg import RelocalizationResult
from landfall_msgs.srv import Relocalize
import launch
import launch.actions
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy

WORK = tempfile.mkdtemp(prefix='landfall_relocalizer_launch_')
LAUNCH = os.path.join(os.environ['LANDFALL_BRINGUP_SHARE'], 'launch', 'relocalizer.launch.xml')
NAMESPACE = 'landfall_relocalizer_launch_test'


@pytest.mark.launch_test
def generate_test_description():
    relocalizer = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource(LAUNCH),
        launch_arguments={
            'database': os.path.join(WORK, 'missing.landfall.db'),
            'startup_relocalization': 'true',
            'namespace': NAMESPACE,
        }.items())
    return launch.LaunchDescription([relocalizer, launch_testing.actions.ReadyToTest()])


class TestRelocalizerLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('relocalizer_launch_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_relocalizer_active_and_answers(self):
        self.assertTrue(wait_until_active(self.node, f'{NAMESPACE}/landfall_relocalizer', 90.0))
        client = self.node.create_client(
            Relocalize, f'/{NAMESPACE}/landfall_relocalizer/relocalize')
        response = call(self.node, client, Relocalize.Request(), 20.0)
        self.assertIsNotNone(response)
        self.assertEqual(response.result.status, RelocalizationResult.NOT_READY,
                         response.result.message)
