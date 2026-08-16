include "cartographer_2d_mapping_imu_lidar_no_odom.lua"


-- ============================================================
-- PURE LOCALIZATION
-- ============================================================

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}


-- ============================================================
-- CARTOGRAPHER OPTIONS
-- ============================================================

options.tracking_frame = "imu_link"
options.published_frame = "base_link"
options.map_frame = "map"

options.provide_odom_frame = true
options.use_odometry = true
options.use_pose_extrapolator = true


-- ============================================================
-- PUBLISH
-- ============================================================

-- 50 Hz
options.pose_publish_period_sec = 0.02

-- 기존값 유지
options.submap_publish_period_sec = 2.0


-- ============================================================
-- SENSOR SAMPLING
-- ============================================================

-- Odom = 50 Hz
options.odometry_sampling_ratio = 1.0

-- IMU = 100 Hz
options.imu_sampling_ratio = 1.0

-- LiDAR = 40 Hz
options.rangefinder_sampling_ratio = 1.0

options.landmarks_sampling_ratio = 0.0
options.fixed_frame_pose_sampling_ratio = 0.0


-- ============================================================
-- LIDAR SCAN SUBDIVISION
-- ============================================================

-- LiDAR 40 Hz
--
-- 한 LaserScan을 10개의 point cloud로 subdivision.
-- 회전 중 scan motion distortion / unwarping 대응.
--
-- 40 Hz → 25 ms / scan
-- 10 subdivisions → 약 2.5 ms 단위
-- ============================================================

options.num_subdivisions_per_laser_scan = 10


-- ============================================================
-- MAP BUILDER
-- ============================================================

-- Jetson Orin Nano 8GB
MAP_BUILDER.num_background_threads = 2


-- ============================================================
-- TRAJECTORY BUILDER 2D
-- ============================================================

TRAJECTORY_BUILDER_2D.use_imu_data = true


-- IMU = 100 Hz
-- 기존값 유지
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 80.0


-- ============================================================
-- RANGE
-- ============================================================

TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 25.0


-- 기존값 유지
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.10


-- ============================================================
-- RANGE DATA ACCUMULATION
-- ============================================================

-- num_subdivisions_per_laser_scan = 10
--
-- subdivision된 range data를 10개 누적하여
-- scan matching용 point cloud 구성.
--
-- 두 값을 10 / 10으로 맞춤.
-- ============================================================

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10


-- ============================================================
-- ONLINE CORRELATIVE SCAN MATCHING
-- ============================================================

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true


-- ------------------------------------------------------------
-- Linear Search
-- ------------------------------------------------------------

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.45


-- ------------------------------------------------------------
-- Angular Search
--
-- 기존: 14°
-- 변경: 18°
--
-- 회전 구간에서 초기 pose prediction 주변을
-- 조금 더 넓게 탐색하도록 설정.
-- ------------------------------------------------------------

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(18.)


-- ------------------------------------------------------------
-- Translation / Rotation Cost
--
-- 기존값 그대로 유지
-- ------------------------------------------------------------

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 8.0

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 16.0


-- ============================================================
-- CERES SCAN MATCHER
-- ============================================================

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 36.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 6.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 48.0


-- 기존값 유지
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 8


-- ============================================================
-- MOTION FILTER
-- ============================================================

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.04

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.8)


-- ============================================================
-- SUBMAP
-- ============================================================

-- Pure Localization이므로 기존값 유지
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40


-- ============================================================
-- POSE GRAPH
-- ============================================================

POSE_GRAPH.optimize_every_n_nodes = 20


-- Global constraint search 기존 설정 유지
POSE_GRAPH.global_sampling_ratio = 0.0


-- ============================================================
-- CONSTRAINT BUILDER
-- ============================================================

POSE_GRAPH.constraint_builder.sampling_ratio = 0.08

POSE_GRAPH.constraint_builder.min_score = 0.68

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85


-- ============================================================
-- FAST CORRELATIVE SCAN MATCHER
-- ============================================================

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.2

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(12.)


-- ============================================================
-- GLOBAL CONSTRAINT SEARCH
-- ============================================================

POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9


-- ============================================================
-- OPTIMIZATION PROBLEM
-- ============================================================

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5

POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5


-- 휠 오돔은 '속도'로만 쓴다 (절대 위치는 라이다/맵만).
-- 이 두 weight가 pose graph 백엔드로 들어가는 유일한 통로이고,
-- 휠 종방향 스케일 오차(실측 4~5% 과다)가 위치추정에 새는 곳도 여기다.
-- "맵핑한 걸로 위치추정하면 직선 길이가 다르다"의 직접 원인.
-- 0으로 잠가도 pose_extrapolator는 odom '속도'를 그대로 쓴다(use_odometry=true).
--
-- 주의: 베이스(맵핑 lua)가 이미 0/0인데 여기서 1e4/1e4로 되살려 놓았었다.
-- 베이스가 "자이로를 두 번 잠근다"는 이유로 rotation을 0으로 둔 결정을
-- 근거 없이 뒤집은 것이라 베이스 결정을 따르도록 되돌린다.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0


-- ============================================================
-- CERES OPTIMIZATION
-- ============================================================

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 4

POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2

POSE_GRAPH.max_num_final_iterations = 4


-- ============================================================
-- RETURN
-- ============================================================

return options