include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "imu_link",  --Key difference: tracking_frame = "imu_link" makes Cartographer integrate IMU in the IMU’s native frame (best practice for 3D).
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,  --3D mode: don’t project pose to 2D; keep roll/pitch/yaw.

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,    --Fuse odometry; GNSS/landmarks off.

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,     --3D expects point clouds (e.g., /points2), not 2D scans.

  -- required by your Cartographer build
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,   --Same sampling ratios; IMU is crucial in 3D.

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.05, --frequency at which its publishing
}

MAP_BUILDER.use_trajectory_builder_3d = true    --Switch to 3D pipeline.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 3  --Accumulate 3 point clouds into one “insertion” to improve SNR and reduce CPU.
TRAJECTORY_BUILDER_3D.min_range = 0.3
TRAJECTORY_BUILDER_3D.max_range = 20.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05    --30 cm min, 20 m max, 5 cm voxel grid for downsampling.

POSE_GRAPH.optimize_every_n_nodes = 120
POSE_GRAPH.constraint_builder.min_score = 0.70
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75  --3D constraints typically need higher scores due to higher scan dimensionality and to avoid false matches.

options.tracking_frame = "imu_link"   -- you can switch to base_link later if IMU is noisy

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 3

POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75

TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds    = 0.8
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians   = 0.02    --3D motion filter a bit looser in time/distance (0.8 s, 20 cm).

POSE_GRAPH.optimize_every_n_nodes = 120   --Reapplies optimize_every_n_nodes.
MAP_BUILDER.collate_by_trajectory = true    --Important for multi-trajectory setups (e.g., separate IMU and LiDAR trajectories).


return options
