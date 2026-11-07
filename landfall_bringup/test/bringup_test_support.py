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

"""Shared by landfall_bringup's launch tests."""

import os
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy


def write_free_map(directory):
    """Write a 1 m x 1 m free occupancy map (map_server YAML and PGM); return the YAML's path."""
    with open(os.path.join(directory, 'map.pgm'), 'wb') as f:
        f.write(b'P5\n20 20\n255\n' + bytes([254]) * 400)
    path = os.path.join(directory, 'map.yaml')
    with open(path, 'w') as f:
        f.write('image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return path


def call(node, client, request, timeout):
    """Call a service; the response, or None if it isn't there or doesn't answer in time."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def wait_until_active(node, name, timeout):
    """Whether lifecycle node `name` reaches the active state within `timeout` seconds."""
    client = node.create_client(GetState, f'/{name}/get_state')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = call(node, client, GetState.Request(), 5.0)
        if response is not None and response.current_state.id == State.PRIMARY_STATE_ACTIVE:
            return True
        time.sleep(0.5)
    return False
