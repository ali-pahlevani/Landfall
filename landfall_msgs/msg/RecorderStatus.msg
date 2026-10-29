# State of the keyframe recorder, published (latched) on its ~/status topic.

builtin_interfaces/Time stamp
string session
bool recording
bool paused

uint32 frames_seen
uint32 keyframes

# Frames rejected by each gate since the session started.
uint32 rejected_pose_uncertain
uint32 rejected_no_motion
uint32 rejected_speed
uint32 rejected_tilt
uint32 rejected_blur
uint32 rejected_exposure
uint32 rejected_tf

geometry_msgs/PoseStamped last_keyframe_pose   # camera pose in the global frame
