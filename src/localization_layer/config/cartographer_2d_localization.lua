-- 위치추정: 22m 직선 끝벽 정합 + 7m/s. 창은 복도 반폭(0.5m)을 넘기지 않음.
include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- 7m/s에서 스캔 1~2장 밀려도(~0.18~0.36m) 벽에 붙게. 0.50 이상은 반대 벽.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.48
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 4.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 38.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 65.0

-- ~40Hz면 40노드 ≈ 1s ≈ 7m. 22m 직선에서 끝벽 제약을 너무 드물게 안 넣음.
POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.max_num_final_iterations = 4
POSE_GRAPH.constraint_builder.max_constraint_distance = 12.
-- 루프클로저도 복도 폭을 넘기지 않음. 8m 떨어진 반대 직선과 혼동하지 않게.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.65
POSE_GRAPH.constraint_builder.min_score = 0.70
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 140

return options
