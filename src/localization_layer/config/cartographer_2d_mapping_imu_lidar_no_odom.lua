include "map_builder_mapping.lua"
include "trajectory_builder.lua"

-- 실차 맵핑: 일자 덕트는 LiDAR degeneracy가 커서
-- 휠+IMU로 x/yaw를 붙들고, 라이다는 벽 정렬(좌우) 담당.
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
  -- 스캔 사이 회전/후진 예측. 최종 맞춤은 아래 ceres 가중치.
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  -- 10이면 40Hz 스캔이 370Hz로 쪼개지고 Jetson pose graph 큐가 1000+로 쌓임.
  -- subdivision 조각 타임스탬프가 다음 스캔과 겹치면 점까지 버림.
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
MAP_BUILDER.num_background_threads = 3

TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 기본 10초라 지속적인 원 운동(원돌이) 중 구심가속도로 쏠린 가속도계 "중력" 방향에
-- 오래 끌려가며 헤딩이 계속 새는 원인이었음. 60초도 코너 한 번에 일부가 새서
-- 자이로를 더 오래 신뢰 (localization은 80).
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 150.0
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 16.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.07
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 회전 직후 IMU prior가 몇 도 어긋나도 코너 벽을 다시 잡게 각 탐색만 넓힘.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.16
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0
-- 효과 있던 prior. 라이다는 벽(occupied)만 강하게.
-- rotation_weight 40은 IMU yaw를 너무 세게 붙잡아 코너에서 라이다가 헤딩을
-- 못 고침. 직선 벽은 원래 yaw를 잘 묶으니 20이면 충분.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 26.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 12.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.08
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)
-- 40은 한 장에 오차가 오래 굳음. 조금 줄여 직전 맵과 더 자주 겹침.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 28
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.46

-- 마지막에 맵이 통째로 한 장 더 그려지는 건 로컬 드리프트보다
-- 비슷한 벽에 잘못된 constraint가 붙고 pose graph가 한 번에 당긴 것.
POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.04
POSE_GRAPH.constraint_builder.max_constraint_distance = 6.0
POSE_GRAPH.constraint_builder.min_score = 0.72
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.80
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.8
-- 6°면 코너 드리프트가 그 이상일 때 로컬 constraint가 못 붙음.
-- 30°(유니스트)는 비슷한 벽에 잘못 걸려 맵이 통째로 돌아가서 12°만.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(12.)
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
-- /odom yaw는 같은 IMU 자이로를 한 번 더 적분한 값. pose graph에 넣으면
-- 코너에서 샌 헤딩이 두 번 잠김. 유니스트도 rotation_weight=0.
-- use_odometry=true라 예측(초기값)에는 그대로 쓰이고, 그래프 residual만 뺌.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 12
POSE_GRAPH.max_num_final_iterations = 80

return options
