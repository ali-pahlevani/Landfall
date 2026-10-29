# One pose hypothesis considered during a relocalization request (diagnostics only).

uint8 STAGE_RETRIEVED=0
uint8 STAGE_SOLVED=1
uint8 STAGE_SANITY_PASSED=2
uint8 STAGE_SCAN_PASSED=3

geometry_msgs/Pose pose        # base pose in the global frame
uint32 cluster_id
uint32 num_inliers
float32 inlier_ratio
uint32[] keyframe_ids          # database keyframes that produced this hypothesis
uint8 stage_reached
string rejection_reason        # empty if not rejected
