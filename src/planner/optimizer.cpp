#include "planner/optimizer.h"

#include <cmath>

namespace planner {

Optimizer::Optimizer()
    : iterations_(10), smooth_weight_(0.1), data_weight_(0.5) {}

void Optimizer::setCostmap(const grid::GlobalCostmap *costmap) {
  costmap_ = costmap;
}

void Optimizer::setWeights(double smooth, double data) {
  smooth_weight_ = smooth;
  data_weight_ = data;
}

void Optimizer::setIterations(int iterations) { iterations_ = iterations; }

std::vector<geometry::Pose2d>
Optimizer::getSmoothPath(std::vector<geometry::Pose2d> &plan) const {
  if (plan.size() < 3)
    return plan;

  std::vector<geometry::Pose2d> new_path = plan;

  for (int iter = 0; iter < iterations_; iter++) {
    double change = 0.0;

    for (size_t i = 1; i < plan.size() - 1; i++) {
      double x_i_x = plan[i].x;
      double y_i_x = new_path[i].x;
      double y_prev_x = new_path[i - 1].x;
      double y_next_x = new_path[i + 1].x;

      double new_x = y_i_x + data_weight_ * (x_i_x - y_i_x) +
                     smooth_weight_ * (y_next_x + y_prev_x - 2.0 * y_i_x);

      double x_i_y = plan[i].y;
      double y_i_y = new_path[i].y;
      double y_prev_y = new_path[i - 1].y;
      double y_next_y = new_path[i + 1].y;

      double new_y = y_i_y + data_weight_ * (x_i_y - y_i_y) +
                     smooth_weight_ * (y_next_y + y_prev_y - 2.0 * y_i_y);

      if (costmap_->getCostAt(new_x, new_y) >=
          (grid::GlobalCostmap::INSCRIBED_COST - 1))
        continue;

      change += std::abs(new_x - new_path[i].x);
      change += std::abs(new_y - new_path[i].y);

      new_path[i].x = new_x;
      new_path[i].y = new_y;
    }

    if (change < epsilon_)
      break;
  }

  return new_path;
}

} // namespace planner
