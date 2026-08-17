-- Cartographer ROS backpack_2d.lua 기본값.
-- 아래만 이 차량 센서 연결용 (튜닝 아님):
--   tracking_frame = imu_link   (use_imu_data 기본 true → IMU 프레임 필수)
--   num_laser_scans = 1         (/scan LaserScan)
--   use_odometry = true         (/odom)

include "map_builder_mapping.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  -- 기본 10은 backpack 데스크류. 젯슨+40Hz면 콜레이터 기아.
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 트랙 ~22m x 8m, 복도 폭 ~1m, 최고 7m/s.
-- 스캔마다 바로 처리 (모아 쓰면 7m/s에서 종방향이 한 번에 밀림)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 80.0
-- 긴 직선 끝벽 정합용. 22m보다 조금 여유.
TRAJECTORY_BUILDER_2D.max_range = 22.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
-- 1m 복도: 0.10이면 횡방향 셀이 너무 굵음. 0.08은 CPU/정합 타협.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.08
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.4
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 160
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 22.0
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 22.0
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100

-- 7m/s·40Hz ≈ 0.18m/스캔. 창이 0.5m 넘으면 1m 복도에서 반대 벽에 붙음.
-- yaw 창은 좁게: 코너는 IMU, 라이다는 헤딩을 거의 안 건드림.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.42
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(8.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 8.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 28.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 32.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 6
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- 7m/s면 0.04m는 한 스캔 안에 넘어가므로 사실상 매 스캔 사용
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.04
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.8)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

return options
