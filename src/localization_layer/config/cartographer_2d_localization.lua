include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 누적 드리프트 완화: prior에 과도하게 묶이지 않고 매 스캔 LiDAR 미세보정.
-- 전역 매칭은 부드럽게·더 자주 (폭력적 스냅백은 피하면서 천천히 틀어지는 것 보정).
options.tracking_frame = "imu_link"
options.published_frame = "base_link"
options.map_frame = "map"
options.provide_odom_frame = true
options.use_odometry = true
options.use_pose_extrapolator = true
options.pose_publish_period_sec = 0.02
options.submap_publish_period_sec = 1.0
options.odometry_sampling_ratio = 1.0
options.imu_sampling_ratio = 1.0
options.rangefinder_sampling_ratio = 1.0
options.landmarks_sampling_ratio = 0.0
options.fixed_frame_pose_sampling_ratio = 0.0

MAP_BUILDER.num_background_threads = 6

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 16.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.08
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.30
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(22.)
-- prior stickiness 완화 → 작은 오차를 스캔마다 깎음
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 8.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 35.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.8)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40

POSE_GRAPH.optimize_every_n_nodes = 30
-- 0.003은 너무 낮아서 전역 재정합 기회 자체가 거의 없었음 → 로컬 탐색이 놓친
-- 누적 편향(돌수록 점점 밀림)을 회수할 안전망이 사실상 꺼져 있던 셈.
-- global_localization_min_score(0.72, 엄격)는 그대로라 엉뚱한 곳 스냅 위험은 낮음.
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.72
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(25.)
POSE_GRAPH.global_constraint_search_after_n_seconds = 6.
-- 2e5였던 건 loop-closure 제약(기본 ~1.1e4/1e5)보다 더 세게 로컬 궤적에 붙잡아서
-- 좋은 매칭이 잡혀도 절반만 당겨지고 편향이 남아 계속 커지는 원인이었음 → 기본값 수준으로.
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
-- odom은 보조만: 누적 스케일 오차가 포즈그래프를 밀지 않게
POSE_GRAPH.optimization_problem.odometry_translation_weight = 8e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 3e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 6
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.max_num_final_iterations = 6

return options
