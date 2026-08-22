-- 로컬라이제이션: CPU 절약 + 코너(고속 yaw) 안정화 균형 프로파일 (2026-08-22).
-- local SLAM(실시간 correlative+ceres)이 즉시 보정, pose graph는 가볍게.
include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

options.pose_publish_period_sec = 5e-3

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- linear 넓히지 않음(S자 평행벽). angular 16°: 18°보다 CPU↓, 12°보다 고속 회전 여유.
-- CPU ≈ angular후보 × linear후보 × 점수함수.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.22
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(16.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 13.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 11.

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 26.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 12.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 63.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 8

-- 반대편 벽(루프) 제거 — 코너 오매칭↓. voxel 0.10 → 점수↓ CPU↓ (0.08 대비 ~20%).
TRAJECTORY_BUILDER_2D.max_range = 18.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.10
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 18.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.20

-- motion_filter: 3/0.04 = 스캔 과다·CPU↑ 방지. max_angle로 고속 회전 구간만 추가 노드.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.2)

MAP_BUILDER.num_background_threads = 2

-- pose graph: 3노드마다는 CPU 폭주. local SLAM이 매칭, PG는 잔차만 가끔 정리.
POSE_GRAPH.optimize_every_n_nodes = 8
POSE_GRAPH.max_num_final_iterations = 3
POSE_GRAPH.constraint_builder.sampling_ratio = 0.22
POSE_GRAPH.constraint_builder.min_score = 0.68
POSE_GRAPH.constraint_builder.max_constraint_distance = 12.
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2.5e4
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 2.2e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4

POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e6

return options
