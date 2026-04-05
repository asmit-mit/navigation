namespace planner {

struct HybridAstarParams {
  double max_angular_velocity = 1.5;
  double max_linear_velocity = 2;

  double angular_resolution = 5;

  double angular_tolerance = 0.1;
  double distance_tolerance = 0.5;

  double analytical_expansion_ratio = 3.5;
  double analytical_expansion_max_length = 3.0;

  double steering_penalty = 0.5;
  double change_steering_penalty = 0.1;
  double reverse_penalty = 2.0;
  double cost_penalty = 6.0;
  double expansion_cost = 200.0;
  double path_length_weight = 0.985;

  int max_explore_iterations = 50000;
};

}; // namespace planner
