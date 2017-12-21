-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link", -- imu_link
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,--true,
  use_odometry = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.01,--0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 5e-3,--30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

--12.4--
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 0.5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 10
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher = {
    linear_search_window = 0.15,
    angular_search_window = math.rad(1.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
}
--12.4--

--12.6--
TRAJECTORY_BUILDER_3D.min_range = 1.0
TRAJECTORY_BUILDER_3D.max_range = 150
TRAJECTORY_BUILDER_3D.submaps.high_resolution = .15
TRAJECTORY_BUILDER_3D.submaps.low_resolution = .55
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 300
POSE_GRAPH.constraint_builder.max_constraint_distance = 100.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 100.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 30.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(60.)
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 200

 
--12.6--

--12.7--
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 9.798
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 20.

--12.7--

--12.8--
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.77
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 60.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 20.
--POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
--POSE_GRAPH.optimization_problem.consecutive_node_rotation_weight = 1e3

--12.8--

--12.11--
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.4
--12.11--

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1--160

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 30--MAP_BUILDER.num_background_threads = 7

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 80--320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200--10
POSE_GRAPH.constraint_builder.min_score = 0.45
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8

return options
