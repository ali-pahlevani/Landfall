# Outcome of one relocalization request. Returned by the Relocalize service and published
# (latched) on the relocalizer's ~/status topic.

uint8 SUCCESS=0
uint8 AMBIGUOUS=1              # two or more well-separated places fit; nothing published
uint8 NO_MATCH=2               # no hypothesis reached the minimum inlier count
uint8 REJECTED_SANITY=3        # failed height, tilt, inlier, coverage or free-space checks
uint8 REJECTED_SCAN_CHECK=4    # passed vision checks, rejected by the LiDAR check
uint8 REJECTED_CONSISTENCY=5   # frames disagreed with each other or with odometry
uint8 NOT_READY=6              # database, map, TF, camera_info, scan or AMCL not ready, or map mismatch
uint8 BUSY=7                   # another request is already running

builtin_interfaces/Time stamp
uint8 status
string trigger                 # "service", "startup" or "auto"
string message                 # human-readable reason
bool published                 # true if the pose was actually sent to AMCL

# Base pose in the global frame. Filled only when status == SUCCESS.
geometry_msgs/PoseWithCovarianceStamped pose

landfall_msgs/RelocalizationScores scores
landfall_msgs/Hypothesis[] hypotheses    # diagnostics, best first
