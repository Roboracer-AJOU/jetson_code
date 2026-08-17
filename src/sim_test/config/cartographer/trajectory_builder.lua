-- Cartographer trajectory_builder.lua 기본값.
include "trajectory_builder_2d.lua"
include "trajectory_builder_3d.lua"

TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,
  collate_fixed_frame = true,
  collate_landmarks = false,
}

return TRAJECTORY_BUILDER
