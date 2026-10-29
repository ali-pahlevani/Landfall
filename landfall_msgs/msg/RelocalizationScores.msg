# Scores of the winning hypothesis (worst value across the frames used).

uint32 frames_used
uint32 num_inliers
float32 inlier_ratio
float32 mean_reprojection_error_px
float32 height_error_m
float32 tilt_error_deg
float32 inlier_coverage                  # fraction of image grid cells holding inliers
float32 ambiguity_ratio                  # second-best / best cluster score; 0 if no competitor
float32 scan_hit_ratio
float32 scan_violation_ratio
float32 consistency_translation_error_m
float32 consistency_rotation_error_deg
float32 elapsed_s
