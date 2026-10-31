# depot_small test fixture

A short cut of the M2 depot recording, used by `test_fixture_build` (the offline pipeline in CI)
and by `test_map_server_parity`.

| File | Content |
|---|---|
| `bag/` | rosbag2, sqlite3, written by Humble's rosbag2 so that every supported distro reads it (4.9 MB) |
| `build.yaml` | `landfall build --config` for this bag |
| `depot.yaml`, `depot.pgm` | nav2_bringup's depot map (Nav2 1.3.13, Apache-2.0), the map AMCL used |

The bag holds two 36 s windows of the recording (recording time 126-162 s and 392-428 s), the
same L-shaped stretch of the depot's open area driven in opposite directions:

| Topic | Rate in the fixture |
|---|---|
| `/rgbd_camera/image/compressed` | every 8th image (about 1.25 Hz), 640 x 480 JPEG re-encoded at quality 60 |
| `/rgbd_camera/camera_info` | with the images |
| `/tf` | every 2nd message (odom->base 25 Hz; map->odom from AMCL) |
| `/tf_static` | the one message of the recording |
| `/amcl_pose` | all |
| `/scan` | 2 Hz |
| `/imu` | 10 Hz |
| `/ground_truth/pose` | 10 Hz, Gazebo's model pose in its world frame (map = world + (8, 0)) |

Made with:

```bash
# Recording (docs/recording.md), inside the simulation container:
scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/record_depot.sh /data/depot_run1'
# Cut on Jazzy (where the recording was made), write the bag on Humble:
python3 landfall_tools/fixtures/depot_fixture.py export --bag depot_run1/bag --out dump
python3 landfall_tools/fixtures/depot_fixture.py write --dump dump --bag bag
```

The images are renders of the Depot model by Open Robotics (Gazebo Fuel, CC-BY-4.0); see
`THIRD_PARTY_LICENSES.md`.
