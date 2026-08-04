include "map_builder_mapping.lua"
include "trajectory_builder.lua"

-- 실차 맵핑: LiDAR 주력 + wheel odom 약한 보조 + IMU(자이로) 보조.
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- IMU는 base_link와 위치가 달라(translation offset) tracking_frame이 될 수 없음
  -- (Cartographer가 IMU-tracking_frame 동일 위치를 강제함). imu_link를 추적하고
  -- published_frame(base_link)은 기존 static TF로 내부 변환.
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.5,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 0.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 0.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 기본 10초라 지속적인 원 운동(원돌이) 중 구심가속도로 쏠린 가속도계 "중력" 방향에
-- 오래 끌려가며 헤딩이 계속 새는 원인이었음 → 60초로 늘려 자이로 적분을 더 오래 신뢰.
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 60.0
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 30.0
-- 짧게: no-return ray로 free를 과하게 깔지 않음 (얇은 벽 끊김 완화)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.07
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.20
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0
-- LiDAR scan match 주력
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 25.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 8.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- 40Hz LiDAR: motion filter 완화 → 안쪽 벽을 더 많은 각도에서 관측
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.8)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
-- 벽(hit)은 빨리 굳히고, free(miss)는 덜 aggressive
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.46

POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.06
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.0
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(12.)
POSE_GRAPH.global_sampling_ratio = 0.0
-- 로컬 SLAM(라이다) 포즈 유지력 >> 휠 odom (특히 rotation은 약하게)
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 12
POSE_GRAPH.max_num_final_iterations = 80

return options
