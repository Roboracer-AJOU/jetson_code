-- Cartographer map_builder.lua 기본값.
include "pose_graph_mapping.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  -- Cartographer는 CPU 0-4. 콜레이터(스캔 입력)를 굶기지 않으려면
  -- 백그라운드 최적화는 2스레드만. 4~7이면 로컬 SLAM 입력이 13Hz로 떨어진다.
  num_background_threads = 2,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}

return MAP_BUILDER
