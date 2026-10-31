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

"""Cut the depot test fixture out of a simulation recording (docs/recording.md).

Two steps, because Humble's rosbag2 cannot read bags written by Jazzy's (metadata version 9)
while every newer rosbag2 reads Humble's:

  export  run on Jazzy, where the recording was made: keeps the given time windows, throttles
          and re-encodes the camera images, and dumps the serialized messages to a directory;
  write   run on Humble: writes the dump as a sqlite3 bag.

The messages involved serialize the same way on every supported distro.

    python3 depot_fixture.py export --bag BAG --out DUMP [--windows 126:162,392:428]
    python3 depot_fixture.py write --dump DUMP --bag FIXTURE
"""

import argparse
import json
import os
import sys

import cv2
import numpy
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage

IMAGE_TOPIC = '/rgbd_camera/image/compressed'

# topic -> keep every n-th message inside the windows (1: all)
TOPICS = {
    '/tf': 2,
    '/amcl_pose': 1,
    IMAGE_TOPIC: 8,
    '/rgbd_camera/camera_info': 8,
    '/scan': 5,
    '/imu': 20,
    '/ground_truth/pose': 5,
}
# Kept whatever its time: published once, at startup.
STATIC_TOPICS = ['/tf_static']


def open_reader(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=''),
        rosbag2_py.ConverterOptions('', ''))
    return reader


def export(args):
    windows = []
    for window in args.windows.split(','):
        start, end = (float(v) for v in window.split(':'))
        windows.append((int(start * 1e9), int(end * 1e9)))
    reader = open_reader(args.bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in list(TOPICS) + STATIC_TOPICS if t not in types]
    if missing:
        sys.exit('missing topics in %s: %s' % (args.bag, ', '.join(missing)))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS) + STATIC_TOPICS))
    os.makedirs(args.out, exist_ok=False)
    counters = {t: 0 for t in TOPICS}
    index = {'topics': {t: types[t] for t in list(TOPICS) + STATIC_TOPICS}, 'messages': []}
    with open(os.path.join(args.out, 'data.bin'), 'wb') as data:
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            if topic not in STATIC_TOPICS:
                if not any(start <= stamp < end for start, end in windows):
                    continue
                counters[topic] += 1
                if (counters[topic] - 1) % TOPICS[topic] != 0:
                    continue
            if topic == IMAGE_TOPIC:
                message = deserialize_message(raw, CompressedImage)
                image = cv2.imdecode(numpy.frombuffer(bytes(message.data), numpy.uint8),
                                     cv2.IMREAD_COLOR)
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                if not ok:
                    sys.exit('cannot re-encode an image')
                message.data = jpeg.tobytes()
                raw = serialize_message(message)
            index['messages'].append([topic, stamp, data.tell(), len(raw)])
            data.write(raw)
    with open(os.path.join(args.out, 'index.json'), 'w') as f:
        json.dump(index, f)
    kept = {}
    for topic, _, _, _ in index['messages']:
        kept[topic] = kept.get(topic, 0) + 1
    print('exported %d messages to %s: %s' % (len(index['messages']), args.out, kept))


def write(args):
    with open(os.path.join(args.dump, 'index.json')) as f:
        index = json.load(f)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, type_name in sorted(index['topics'].items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=type_name, serialization_format='cdr'))
    with open(os.path.join(args.dump, 'data.bin'), 'rb') as data:
        for topic, stamp, offset, length in index['messages']:
            data.seek(offset)
            writer.write(topic, data.read(length), stamp)
    del writer
    print('wrote %d messages to %s' % (len(index['messages']), args.bag))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('export', help='run on Jazzy: dump the fixture messages')
    p.add_argument('--bag', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--windows', default='126:162,392:428',
                   help='start:end in recording time (seconds), comma separated')
    p.add_argument('--quality', type=int, default=60, help='JPEG quality of the images')
    p = commands.add_parser('write', help='run on Humble: write the dump as a sqlite3 bag')
    p.add_argument('--dump', required=True)
    p.add_argument('--bag', required=True)
    args = parser.parse_args()
    if args.command == 'export':
        export(args)
    else:
        write(args)


if __name__ == '__main__':
    main()
