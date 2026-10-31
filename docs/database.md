# Map database

A Landfall map is one SQLite file, by convention `<name>.landfall.db`. It is written by the offline
builder (`landfall build`, see `docs/recording.md`) and opened read-only by the relocalizer.

## Identity and versioning

- `PRAGMA application_id = 0x4C444641` ('LDFA'). Any other value, or a file that isn't SQLite, is
  refused as "not a Landfall database".
- `PRAGMA user_version` is the schema version, currently **1**. A database older than the running
  Landfall must be migrated with `landfall migrate` (which first copies the file to
  `<file>.bak-v<N>`); one newer than the running Landfall is refused. The relocalizer never
  migrates.
- Rollback journal, no WAL: a read-only open never needs `-wal`/`-shm` side files.
- `foreign_keys = ON` on every connection.

## Encoding

- Blobs hold raw little-endian values (x86_64 and aarch64 are both little-endian; the build fails
  on a big-endian host).
- Poses are row-major 3×4 matrices `[R | t]` (12 × f64), so they round-trip bit-exactly.
- Keypoints are 5 × f32 each: x, y, size, angle, score (pixels of the distorted image).
- Descriptor blobs carry their element type: 0 = u8, 1 = f32, 2 = f16, 3 = sparse (u32 index,
  f32 value) pairs in ascending index order, for bag-of-words vectors (`dim` = number of pairs).
- `maps.grid` is a lossless 8-bit PNG of the occupancy values plus one (unknown −1 → 0), top row
  first.

## Tables

| Table | Key columns | Content |
|---|---|---|
| `meta` | `key` | `schema_version`, `landfall_version`; the builder adds its configuration, report and `content_sha256` |
| `maps` | `id` | map YAML text and SHA-256, image SHA-256, grid SHA-256, resolution, origin, size, the grid itself |
| `cameras` | `id` | frame id, distortion model, size, intrinsics (fx, fy, cx, cy), distortion coefficients, base→camera pose |
| `sessions` | `id` | one per recording (bag or staging folder) |
| `keyframes` | `id` | session, camera, stamp, prior base pose (x, y, yaw) and covariance, refined pose, map→camera pose, flags, quality JSON, JPEG thumbnail |
| `backends` | `id` | role (`global`, `local`, `matcher`, `vocabulary`), name, version, model SHA-256, canonical config and its SHA-256 |
| `local_features` | `(keyframe_id, backend_id)` | keypoints and descriptors of one keyframe |
| `global_descriptors` | `(keyframe_id, backend_id)` | retrieval descriptor of one keyframe |
| `landmarks` | `id` | 3D point in the map frame, track length, mean reprojection error, largest ray angle, representative descriptor |
| `observations` | `(keyframe_id, keypoint_idx)` | which landmark a keypoint observes (indexed by landmark too) |
| `covisibility` | `(kf_a, kf_b)`, `kf_a < kf_b` | number of shared landmarks |
| `vocabulary` | `backend_id` | serialized vocabulary tree (classical backend) |
| `pca` | `backend_id` | optional projection for learned global descriptors |

## What the builder writes

- `meta`: `landfall_version`, `created_utc`, `builder_config` (every effective setting as YAML),
  `build_report` (the build statistics as JSON, also in `report.json`) and `content_sha256`.
- One `maps` row (the map the build was given), one `cameras` row (v1.0 builds use one camera;
  `T_base_cam` is the first keyframe's), one `sessions` row per staging folder or bag.
- `keyframes`: the recorded pose and covariance (NaN when the pose source has none), the pose after
  the bundle adjustment, `T_map_cam` from the refined pose, the quality metrics as JSON and a
  JPEG thumbnail (at most 160 px). Flag bit 0: BA moved the keyframe to its shift bound.
- `backends`: rows for the local features (`local`, e.g. `orb`), the global descriptor (`global`,
  `bow`, whose config holds the vocabulary's SHA-256), the matcher (`matcher`) and the vocabulary
  (`vocabulary`, `vocab_tree`, whose blob is in `vocabulary`). A backend's `config` is its canonical
  configuration; the relocalizer re-creates the backend from it.
- `local_features`: only the keypoints that observe a landmark (`db.prune_unobserved_features`),
  so `observations.keypoint_idx` indexes this reduced list.
- `landmarks`: position, track length, mean reprojection error, largest ray angle, and the medoid
  of the track's descriptors.
- `covisibility`: keyframe pairs sharing at least `db.covisibility_min_shared` (5) landmarks.

## Content hash

`content_sha256` covers every table except `meta`, rows in primary-key order, each value with its
SQLite type tag. Two builds from the same input with the same seeds produce the same hash.

## Binding to the occupancy map

The relocalizer (M3) will only accept a database together with the map it was built for: the grid size,
resolution and origin must match, and the cells must match the `/map` that nav2_map_server
publishes. nav2_map_server converts pixels differently on Humble (Nav2 1.1) and on Jazzy/Lyrical
(Nav2 ≥ 1.3) for color and transparent images, so the database stores the grid itself and the
loader reproduces both rules.
