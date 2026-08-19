-- 직선에서 TF가 뒤로 뚜둑 밀리는 건 IMU/오돔이 아니라
-- correlative/포즈그래프가 복도에서 더 높은 점수를 뒤쪽으로 집기 때문.
include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- 급가속 직진: 창을 키우면 안 된다. 복도 옆벽은 종방향으로 비슷해서
-- 후보를 넓게 두면 점수 높은 뒤쪽 벽에 붙어 TF가 밀린다.
-- 스캔 간 이동은 오돔 예측이 담당하고, 창은 예측 잔차만 보면 된다.
-- 실측 CPU = 각도후보 x 이동후보 x 점수. (12°, 0.50, 139점) 에서 71%.
-- 0.35 면 이동후보가 441 -> 225 로 줄어 36% 대가 된다.
-- 0.35 는 ±17.5cm 로, 오돔 예측 잔차(수 cm)를 충분히 덮는다.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.22
-- 15°면 각도 후보만 159개다. 38Hz 에서 스캔 간 회전은 3° 남짓이라
-- 12°로도 예측 잔차를 덮는다. 15->12 로 CPU 88% -> 71% (예측 70.4%와 일치).
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.)
-- 예측 pose에서 멀리 떨어진 후보에 페널티. 20은 가속 점프는 막았지만
-- 매칭이 프라이어에 붙어서 차량 이동을 한 박자 늦게 따라갔다.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 18.

-- occupied를 높이면 끝벽은 잘 붙지만, 가속 중엔 옆벽 오매칭으로 종방향이 밀린다.
-- translation_weight는 오돔을 너무 세게 붙잡지 않게 12.
-- 3e4 오돔 가중은 1바퀴 종방향 스케일 오차를 맵이 못 당겨서
-- 2바퀴째 직선이 기존 라인보다 더 나가고 잔차가 남았다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 24.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 14.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 55.

-- 필터는 2단이고 voxel_filter_size 가 먼저다. 실측 861점 -> 139점.
-- adaptive_voxel_filter 는 그 139점을 받는데, 입력이 min_num_points 이하면
-- 카토그래퍼는 그대로 반환한다. 즉 아래 두 줄은 현재 아무 일도 하지 않는다.
-- (min_num_points 800->400, max_length 0.20->0.08 이 CPU 에 무영향이었던 이유)
--
-- 끝벽 보존은 voxel_filter_size 가 결정한다. 20m 에서 빔 간격이 8.4cm 라
-- 0.08 voxel 은 끝벽 점을 하나도 안 버린다. 반대로 1.5m 옆벽은 간격 0.63cm 라
-- 12.7:1 로 솎인다. 즉 이 필터는 이미 먼 점에 유리하게 걸려 있다.
-- 끝벽이 안 잡히는 건 필터 문제가 아니다.
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.08
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 25.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.20
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04

-- 포즈그래프는 전부 백그라운드 스레드에서 돈다. 실측상 이 스레드들은
-- 4/4/2% 로 놀았고 로컬 코어 0-4 중 3개가 비어 있었다. 정합을 빠르게 하려면
-- 여기부터 늘려야 아래 주기가 실제로 지켜진다. 메인 SLAM 스레드는 안 건드린다.
MAP_BUILDER.num_background_threads = 4

-- 맵 정합 주기. 6~7m/s 에서는 모든 스캔이 모션필터를 통과해 노드가 40개/초다.
--   20노드 = 0.50초마다 (2.0회/초)
--   10노드 = 0.25초마다 (4.0회/초)
-- 드리프트가 일정 속도로 쌓이면 한 번의 보정 크기는 주기에 비례한다.
-- 실측(2.9m/s)에서 map->odom 이 10~13cm 씩 튀었으니 절반인 6~7cm 가 된다.
-- 보정이 잘게 들어와야 스탠리가 큰 횡오차를 한 번에 보고 꺾지 않는다.
POSE_GRAPH.optimize_every_n_nodes = 6
POSE_GRAPH.max_num_final_iterations = 4
-- 주기만 줄이면 같은 제약으로 같은 문제를 다시 풀 뿐이다. 정합이 실제로
-- 자주 일어나려면 노드가 고정맵과 맞춰지는 빈도도 같이 올려야 한다.
-- 문턱(min_score)은 그대로 둬서 품질 기준은 낮추지 않는다.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.35
POSE_GRAPH.constraint_builder.min_score = 0.68
POSE_GRAPH.constraint_builder.max_constraint_distance = 12.
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2.5e4
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)
-- 3e4는 가속 점프는 막았지만 2바퀴 잔차를 맵이 못 고쳤다.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 2.2e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4

POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e6

return options