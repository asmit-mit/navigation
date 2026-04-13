#pragma once

namespace controller {

struct ControllerParams {
  double max_linear_velocity = 0.22;
  double max_angular_velocity = 1.0;

  double lookahead_distance = 0.6;
  double lookahead_gain = 1.5;
  double max_lookahead_distance = 0.9;
  double min_lookahead_distance = 0.3;

  double proximity_distance = 0.1;
  double proximity_heurisitc_scale = 1.0;

  double distance_tolerance = 0.5;
};

}; // namespace controller
