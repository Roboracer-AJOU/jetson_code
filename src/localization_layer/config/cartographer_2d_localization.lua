include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- 실차 loc: /odom 50Hz. 회전 깨짐은 라이다 yaw 오매칭.
-- IMU: odom 자이로(이미 /odom에 들어감)와 Cartographer IMU는 다름.
--   Cartographer IMU는 스캔 사이 자세 예측용. tracking은 imu_link여야 함.
-- 전역재탐색: 로컬 매칭(근처 맵)과 다름. 길을 잃었을 때 맵 전체에서 다시 찾음.

options.tracking_frame = "imu_link"
options.published_frame = "base_link"
options.map_frame = "map"
options.provide_odom_frame = true
options.use_odometry = true
options.use_pose_extrapolator = true
options.pose_publish_period_sec = 0.05
options.submap_publish_period_sec = 2.0
options.odometry_sampling_ratio = 1.0
options.imu_sampling_ratio = 1.0
options.rangefinder_sampling_ratio = 1.0
options.landmarks_sampling_ratio = 0.0
options.fixed_frame_pose_sampling_ratio = 0.0

MAP_BUILDER.num_background_threads = 2

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 80.0
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.10
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.35
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 8.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 55.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 6
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.06
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

POSE_GRAPH.optimize_every_n_nodes = 80
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.04
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.82
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(12.)
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2
POSE_GRAPH.max_num_final_iterations = 4

return options
