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

"""Write the images of the parity files, taken from the depot fixture bag.

    python3 -I parity_images.py --bag landfall_ros/test/fixtures/depot_small/bag --output DIR

Writes the fixture's camera images 11, 30 and 60 as they are in the bag (JPEG), and a 600 x 450
crop of image 30 (PNG): a size that isn't a multiple of 32, so that XFeat's resize is compared
too. Prints the paths, one per line. Reads the bag's sqlite3 file directly, so no ROS is needed.
The images are renders of the Depot model by Open Robotics (Gazebo Fuel, CC-BY-4.0; see
THIRD_PARTY_LICENSES.md).
"""

import argparse
import pathlib
import sqlite3
import struct

import cv2
import numpy as np

TOPIC = '/rgbd_camera/image/compressed'
INDICES = (11, 30, 60)
CROP_INDEX = 30
CROP_ROWS = slice(15, 465)
CROP_COLUMNS = slice(20, 620)


def compressed_image_data(blob):
    """The `data` field of a CDR-serialized sensor_msgs/CompressedImage."""
    if bytes(blob[:2]) != b'\x00\x01':
        raise SystemExit('not little-endian CDR')
    body = memoryview(blob)[4:]  # alignment counts from the end of the encapsulation header
    offset = 0

    def u32():
        nonlocal offset
        offset = (offset + 3) & ~3
        (value,) = struct.unpack_from('<I', body, offset)
        offset += 4
        return value

    u32()  # header.stamp.sec
    u32()  # header.stamp.nanosec
    for _ in ('header.frame_id', 'format'):  # strings, their length counting the final null
        length = u32()  # (not `offset += u32()`: that reads offset before u32 moves it)
        offset += length
    size = u32()
    data = bytes(body[offset:offset + size])
    if len(data) != size or not data.startswith(b'\xff\xd8'):
        raise SystemExit('the image is not a JPEG')
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--bag', required=True, type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    args = parser.parse_args()

    files = sorted(args.bag.glob('*.db3'))
    if len(files) != 1:
        raise SystemExit(f'{args.bag}: expected one .db3 file')
    db = sqlite3.connect(f'file:{files[0]}?mode=ro', uri=True)
    topic = db.execute('SELECT id FROM topics WHERE name = ?', (TOPIC,)).fetchone()
    if topic is None:
        raise SystemExit(f'{args.bag}: no {TOPIC}')
    blobs = [row[0] for row in db.execute(
        'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp, id', topic)]
    if len(blobs) <= max(INDICES):
        raise SystemExit(f'{args.bag}: {len(blobs)} images, image {max(INDICES)} needed')

    args.output.mkdir(parents=True, exist_ok=True)
    paths = []
    for index in INDICES:
        path = args.output / f'fixture_{index:03d}.jpg'
        path.write_bytes(compressed_image_data(blobs[index]))
        paths.append(path)
    jpeg = np.frombuffer(compressed_image_data(blobs[CROP_INDEX]), np.uint8)
    crop = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)[CROP_ROWS, CROP_COLUMNS]
    path = args.output / f'fixture_{CROP_INDEX:03d}_crop{crop.shape[1]}x{crop.shape[0]}.png'
    if not cv2.imwrite(str(path), crop):
        raise SystemExit(f'cannot write {path}')
    paths.append(path)
    for path in paths:
        print(path)


if __name__ == '__main__':
    main()
