-- backpack_2d.lua 기반. 센서 연결 + 젯슨에서 필요한 것만.
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

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 80.0
-- 반대편 벽/루프 혼동↓. 40m면 좁은 트랙에서 평행벽 constraint가 직선을 당김.
TRAJECTORY_BUILDER_2D.max_range = 22.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.10

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 직선 복도: 옆벽은 종방향으로 비슷해서 정합 시 뒤쪽 후보가 선택되면
-- 직선 길이가 누적적으로 짧아짐(압축). 창은 odom 예측 잔차(±3~4cm)만.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.04
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(8.)
-- 예측보다 뒤로 붙는 후보(직선 압축)에 강한 페널티. 30+ 는 한 박자 늦게 따라감.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 26.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 28.0

-- occupied↓ + translation↑: degenerate 직선에서 라이다가 종방향을 줄이지 못하게.
-- arc length는 wheel odom+IMU 예측을 우선 유지.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 16.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 32.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04
-- 서브맵 경계가 많을수록 직선 중 inter-submap 정합으로 길이가 줄어들기 쉬움.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

return options
