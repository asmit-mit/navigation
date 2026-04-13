namespace controller {

struct ControllerParams {
  double max_linear_velocity = 2.0;

  double lookahead_distance = 0.6;
  double lookahead_gain = 0.2;
  double max_lookahead_distance = 0.9;
  double min_lookahead_distance = 0.3;
};

}; // namespace controller
