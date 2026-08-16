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
  -- 스캔 디스큐(모션 보정). 20Hz면 한 바퀴가 50ms라 3m/s에서 그동안 차가 15cm를
  -- 가는데, 1이면 그 15cm에 걸쳐 찍힌 점을 전부 한 시각의 점으로 취급해서
  -- /scan이 종방향으로 늘어지거나 땡겨진다. 10으로 쪼개면 조각마다 extrapolator가
  -- 제 시각의 자세를 붙여준다(그래서 odom/IMU stamp 정확도가 여기에 직접 먹힌다).
  --
  -- 아래 num_accumulated_range_data = 10과 반드시 같이 간다. 10조각을 모아 한 번
  -- 매칭하므로 스캔매칭 횟수는 20회/s로 지금과 동일하고, 초당 점 개수도 그대로다.
  -- 늘어나는 건 collator/extrapolator 호출뿐이라 매칭보다 훨씬 싸다.
  -- 예전에 10에서 터진 건 40Hz(400조각/s) 때였고 지금은 20Hz라 200조각/s.
  -- localization lua는 이미 10/10으로 이 Jetson에서 돌고 있음.
  -- 둘 중 하나만 바꾸면 노드 생성률이 10배로 튀니 항상 쌍으로 바꿀 것.
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.5,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 3

TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 기본 10초라 지속적인 원 운동(원돌이) 중 구심가속도로 쏠린 가속도계 "중력" 방향에
-- 오래 끌려가며 헤딩이 계속 새는 원인이었음. 60초도 코너 한 번에 일부가 새서
-- 자이로를 더 오래 신뢰 (localization은 80).
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 120.0
TRAJECTORY_BUILDER_2D.min_range = 0.12
-- 직선 구간 길이가 22m인데 16m로 자르면 앞 끝벽이 안 보여서, 진입 후 6m를
-- 갈 때까지 좌우 옆벽만 남아 종방향 정보가 0이 된다(후방 95°는 차체에 가림).
-- 그 구간에서 x가 순전히 오돔에만 의존해 밀렸음. 센서는 Sensitivity 모드에서
-- 40m까지 보므로(로그 확인) 25m로 올려 시작부터 끝벽을 잡게 함.
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
-- num_subdivisions_per_laser_scan = 10과 쌍. 10조각(=스캔 한 바퀴)을 모아
-- 디스큐된 클라우드 하나로 매칭한다. 노드 생성률은 20Hz로 이전과 같다.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.07
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 회전 직후 IMU prior가 몇 도 어긋나도 코너 벽을 다시 잡게 각 탐색만 넓힘.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.16
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.)
-- 0.2까지 낮췄더니 22m 먼 벽의 성긴/노이즈 있는 매칭이 pose를 종방향으로 끌어
-- /scan이 앞뒤로 땡겨지는 증상이 생김. 원래 값으로 복귀.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0
-- 효과 있던 prior. 라이다는 벽(occupied)만 강하게.
-- rotation_weight 40은 IMU yaw를 너무 세게 붙잡아 코너에서 라이다가 헤딩을
-- 못 고침. 직선 벽은 원래 yaw를 잘 묶으니 20이면 충분.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 26.0
-- 4로 낮췄더니 먼 거리 노이즈에 pose가 끌려다님. 직선의 종방향 단서는 신뢰할
-- 만큼 깨끗하지 않으므로 prior 쪽 규제를 원래대로 유지.
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

POSE_GRAPH.optimize_every_n_nodes = 80
-- 아래 세 값은 예전에 "잘못된 constraint가 붙어 맵이 당겨진다"는 이유로 베이스
-- (pose_graph_mapping.lua)보다 낮춰 덮어썼는데, 그 결과 랩이 쌓일수록 같은 벽이
-- 1~2m 어긋난 복사본으로 겹쳐 그려졌다(20260816 21:18 vs 21:22 저장본 비교).
-- 0.8m 창으로는 1~2m 어긋난 걸 원리적으로 못 찾으므로 로컬 constraint가 아예
-- 안 붙던 것. 오검출 방어는 창 크기가 아니라 min_score(0.72, 기본 0.55)로 건다.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.12
POSE_GRAPH.constraint_builder.max_constraint_distance = 8.0
POSE_GRAPH.constraint_builder.min_score = 0.72
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.0
-- 6°면 코너 드리프트가 그 이상일 때 로컬 constraint가 못 붙음.
-- 30°(유니스트)는 비슷한 벽에 잘못 걸려 맵이 통째로 돌아가서 12°만.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(12.)
-- 전역 탐색: 창 제한 없이 전체 submap을 뒤지므로 비슷하게 생긴 덕트 벽에 잘못
-- 걸리면 맵이 통째로 돌아간다. 기본값 수준으로만 아주 얕게 켜고(0.003 = 노드
-- 1000개당 3개), 문턱은 베이스의 0.82로 올려 방어한다. 맵이 한 번에 홱 도는
-- 증상이 보이면 여기 두 줄만 0.0 / 1e9로 되돌리면 로컬 루프클로저는 유지된다.
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.0
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.82
-- constraint가 실제로 붙는지 로그로 확인 (Added N constraints ... / scores).
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
-- ============================================================
-- 휠 오돔은 '속도'로만 쓴다 (절대 위치는 라이다만)
-- ============================================================
-- Cartographer가 /odom을 쓰는 경로는 둘뿐이다.
--   1) pose_extrapolator: 연속 odom pose의 '차이'로 속도만 뽑는다.
--      절대 위치는 애초에 안 본다. 스캔 사이 50ms 예측용.
--   2) pose graph 백엔드: 연속 노드 간 '상대 변위' constraint로 들어간다.
--      아래 두 weight가 그 통로다.
-- 휠 오돔 종방향 스케일이 실측 4~5% 과다인데(straight_diag), 2)를 열어두면
-- 그 오차가 맵과 위치추정에 그대로 구워진다. 0으로 잠그면 백엔드는 순전히
-- 스캔매칭+루프클로저가 정하고, odom은 1)의 속도 예측으로만 남는다.
-- (unicorn-racing-stack이 EKF에서 wheel odom을 twist(vx,vyaw)만 받는 것과
--  같은 구조. 저쪽은 EKF로, 우리는 이 두 줄로 같은 격리를 얻는다.)
-- use_odometry = true 는 그대로 둬야 한다 — 꺼버리면 1)까지 사라진다.
-- wheel_diameter를 실측 보정한 뒤에는 1e3 정도로 되살릴지 재검토할 것.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0
-- /odom yaw는 같은 IMU 자이로를 한 번 더 적분한 값. 그래프에 넣으면
-- 코너에서 샌 헤딩이 두 번 잠긴다.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 12
POSE_GRAPH.max_num_final_iterations = 80

return options
