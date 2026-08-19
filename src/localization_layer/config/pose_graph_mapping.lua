-- Cartographer pose_graph.lua 기본값 (Humble / upstream 동일).
-- 파일 이름만 pose_graph_mapping.lua — include 경로를 유지하기 위함.

POSE_GRAPH = {
  optimize_every_n_nodes = 80,
  constraint_builder = {
    sampling_ratio = 0.04,
    max_constraint_distance = 15.,
    min_score = 0.65,
    global_localization_min_score = 0.82,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = false,
    fast_correlative_scan_matcher = {
      linear_search_window = 1.2,
      angular_search_window = math.rad(20.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 10.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1.1e2,
    rotation_weight = 1.6e4,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e4,
    odometry_rotation_weight = 1e3,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 8,
      num_threads = 2,
    },
  },
  max_num_final_iterations = 20,
  global_sampling_ratio = 0.003,
  log_residual_histograms = false,
  global_constraint_search_after_n_seconds = 15.,
}

return POSE_GRAPH
