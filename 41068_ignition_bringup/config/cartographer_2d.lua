include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.05,
}

MAP_BUILDER.use_trajectory_builder_2d = true  --Switch Cartographer into 2D SLAM mode.

TRAJECTORY_BUILDER_2D.use_imu_data = false    --2D pipeline ignore IMU. Good for pure LiDAR robots; be aware of reduced robustness on aggressive acceleration/tilt.
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0 --LiDAR ray handling: discard close returns (<15 cm), cap far returns (30 m), and when a ray is missing (no hit) insert a 5 m free-space ray.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025 --Pre-filter scan points into 2.5 cm voxels—noise and bandwidth reduction.

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = 0.02 --Motion filter drops redundant scans if not enough motion occurred since last insertion: <0.5 s, <10 cm, <~1.1°. CPU saver; tune for robot speed.

POSE_GRAPH.optimize_every_n_nodes = 90 --Run global pose graph optimization every 90 node insertions (nodes ≈ scan accumulations). Larger = less CPU, more delayed loop closure.
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60  --Scan-to-submap matching thresholds: lower = more matches (risk of false positives), higher = more conservative.

return options
