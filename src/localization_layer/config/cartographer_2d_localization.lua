include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- Jetson lite @ LiDAR 40Hz + path_following 동시 실행용.
-- CSM/pose graph 부하를 낮춰 Cartographer가 CPU를 독점하지 않게 함.
-- IMU: use_imu_data=true (코너 yaw). Cartographer는 IMU와 tracking_frame이
-- 같은 위치여야 하므로 tracking=imu_link, published=base_link (매핑 lua와 동일).
-- EBIMU가 끊기면 OrderedMultiQueue가 멈추니 /imu/data가 연속인지 확인.
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
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.10
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 작은 창 = 저CPU. 40Hz면 스캔당 이동이 작아서 충분.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.28
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 6
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.06
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

POSE_GRAPH.optimize_every_n_nodes = 80
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.08
POSE_GRAPH.constraint_builder.min_score = 0.58
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
POSE_GRAPH.global_constraint_search_after_n_seconds = 12.
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 5e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2
POSE_GRAPH.max_num_final_iterations = 4

return options
