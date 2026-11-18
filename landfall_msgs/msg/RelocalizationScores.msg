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
float32 closeness                        # how near the request came to a pose: 1 with one, 0 without
                                         # any place (query.early_stop_closeness)
bool stopped_early                       # ended before query.timeout: a still camera came nowhere near
                                         # a pose (query.early_stop_frames)
float32 scan_alternative_gap             # how much better the scan fits the pose than the best place
                                         # beyond 1 m or 30 deg (hit ratio less violation ratio; 1
                                         # without one, negative: it fits there better), smallest
                                         # over the frames; NaN when that search didn't run
