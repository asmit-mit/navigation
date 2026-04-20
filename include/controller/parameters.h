#pragma once

namespace controller {

struct ControllerParams {
  int controller_frequency = 20;
  double max_linear_velocity = 1.0;
  double max_angular_velocity = 2.0;
  double max_linear_acceleration = 1.0;
  double max_angular_acceleration = 1.8;
  double sim_time = 1.0;

  double lookahead_distance = 0.6;
  double lookahead_gain = 1.5;
  double max_lookahead_distance = 0.9;
  double min_lookahead_distance = 0.3;

  double proximity_distance = 0.3;
  double proximity_heurisitc_scale = 1.0;

  double approach_velocity_scaling_dist = 1.0;
  double min_approach_linear_velocity = 0.05;
  double min_heading_angle_error = 0.785;

  double distance_tolerance = 0.1;
};

}; // namespace controller
