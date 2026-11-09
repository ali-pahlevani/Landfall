# Field validation

How to validate Landfall on a real robot before relying on it: the procedure of PLAN M8, written
for any robot. The goal is the release's promise, measured where the robot works: **no wrong
pose** (more than 0.5 m or 10° off), with the success and miss rates and the timings that come with
it. A real site has no ground truth; the reference is the robot's own localization while it is
well localized, checked by the operator wherever the two disagree.

## Before starting

- **Sensors** set up as in `docs/sensors.md`: calibrated camera, measured extrinsics in the URDF,
  `image.time_offset` measured, odometry and LiDAR as AMCL uses them.
- **AMCL** well tuned for the site: the validation compares with it.
- **TF**: every frame of `docs/sensors.md` at the image stamps; a camera on a moving joint must
  have the joint in TF, calibrated.
- **One writer of `/initialpose`.** If another node also sets AMCL's pose (a saved pose restored
  at startup, another relocalization), decide which one publishes during the trials; Landfall can
  stay in `dry_run` while another node publishes.
- **The forklift of PLAN §14** in particular: the fork carriage's TF must come from the calibrated
  joint position (R25), the camera's latency must be measured (R26), and its localization monitor
  must not race the relocalizer (R27).

## 1. Recording

Record at least three sessions on different days or in different lighting, driving every aisle in
both directions while AMCL is well localized (`docs/recording.md`). Build one database from all
of them and read the build report: keyframes per area, the heading rose of every cell (gaps are
directions never driven), landmarks per keyframe.

## 2. Shadow mode

Run the relocalizer beside the robot's normal operation without letting it publish anything:

```yaml
landfall_relocalizer:
  ros__parameters:
    startup.enabled: false
    auto_trigger.enabled: false
    kidnap_watchdog.enabled: false     # it would publish
    extrinsics_monitor.enabled: true
    extrinsics_monitor.period: 10.0    # one dry-run request every 10 s
    extrinsics_monitor.max_amcl_std_xy: 0.10   # at or above AMCL's std when it is right
    shadow.log_path: /var/log/landfall/shadow.csv
```

The extrinsics monitor's requests run every check of a real request and publish nothing; with
`shadow.log_path`, each one becomes a row with its status, its pose and the localization's pose at
the same stamp. Over at least eight operating hours this gives about 2,900 requests. Then:

```bash
ros2 run landfall_benchmark summarize_shadow --max-std-xy 0.10 --list shadow.csv
```

It counts, per trigger, the requests and their statuses, and of the successes taken while the
localization was confident: those that agree with it (within 0.25 m and 5°) and those that
disagree (beyond 0.5 m or 10°), with the 95 % Clopper-Pearson upper bound of the disagreement rate.
`--list` prints the disagreeing rows. Every one needs an explanation: either Landfall was wrong (a
wrong pose: stop, and find why), or the localization was (e.g. a robot pushed by hand); the robot's
bag or the operator's notes tell which.

## 3. Active trials

At least 50 relocalizations at varied spots: along the aisles and across them, at different
headings, near repetitive structures, with and without loads, at the ends of aisles.

1. `dry_run` first: call `~/relocalize` with `dry_run: true` while AMCL is well localized, and
   compare the answer with AMCL's pose (the shadow log records these too).
2. Then live: make AMCL lose its pose (`reinitialize_global_localization`, or a wrong initial pose
   from RViz), call `~/relocalize`, and check that AMCL converges on the right pose (the operator
   confirms where the robot is).

Note every result: status, the pose's error against the confirmed pose, the time.

## 4. Timings

On the robot's own computer, from the shadow log (`elapsed_s`, the whole request) and the node's
log (milliseconds per frame). The release's target for the classical backend is a median request
under 3 s on a CPU-only PC.

## Results to report

| What | How |
|---|---|
| Wrong poses | count over shadow and active trials, with the 95 % upper bound; target 0 |
| Success and miss rates | per trigger, from the shadow log; per spot, from the active trials |
| Timings | median and 95th percentile per request |
| Stability | no crash, no growing memory over the soak |
| Changes | every threshold changed from its default, and why |
