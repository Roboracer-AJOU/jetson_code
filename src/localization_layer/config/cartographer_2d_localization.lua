include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Pure localization: LiDAR only. 휠 odom / IMU 미사용.
options.tracking_frame = "base_link"
options.published_frame = "base_link"
options.map_frame = "map"
options.provide_odom_frame = false
options.use_odometry = false
options.use_pose_extrapolator = true
options.pose_publish_period_sec = 0.02
options.submap_publish_period_sec = 1.0
options.odometry_sampling_ratio = 0.0
options.imu_sampling_ratio = 0.0
options.rangefinder_sampling_ratio = 1.0
options.landmarks_sampling_ratio = 0.0
options.fixed_frame_pose_sampling_ratio = 0.0

MAP_BUILDER.num_background_threads = 3

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.06
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- Aggressive high-speed (near Jetson limit). Watch CPU / scan rate.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.85
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.5
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 14
-- Moving: nearly every scan. Idle: ~33 Hz cap.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 16

POSE_GRAPH.optimize_every_n_nodes = 150
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.constraint_builder.sampling_ratio = 0.015
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 8
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 3
POSE_GRAPH.max_num_final_iterations = 10

return options
