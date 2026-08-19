-- Cartographer map_builder.lua 기본값.
include "pose_graph_mapping.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  -- 기본 4. 젯슨에선 콜레이터 기아 방지로 2.
  num_background_threads = 2,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}

return MAP_BUILDER
