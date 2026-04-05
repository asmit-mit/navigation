#include "planner/optimizer.h"
#include <cmath>
#include <iostream>

using std::cout;
using std::endl;

namespace planner {

Optimizer::Optimizer()
    : iterations_(10), smooth_weight_(0.1), data_weight_(0.5) {}

void Optimizer::setWeights(double smooth, double data) {
  smooth_weight_ = smooth;
  data_weight_ = data;
}

void Optimizer::setIterations(int iterations) { iterations_ = iterations; }

std::vector<Pose2d> Optimizer::getSmoothPath(std::vector<Pose2d> &plan) const {
  if (plan.size() < 3)
    return plan;

  std::vector<Pose2d> new_path = plan;
  std::vector<Pose2d> last_path = plan;

  const double tolerance = 1e-6;

  for (int iter = 0; iter < iterations_; iter++) {
    double change = 0.0;

    for (size_t i = 1; i < plan.size() - 1; i++) {
      double x_i = plan[i].x;
      double y_i = new_path[i].x;
      double y_prev = new_path[i - 1].x;
      double y_next = new_path[i + 1].x;

      double y_i_old = y_i;

      y_i += data_weight_ * (x_i - y_i) +
             smooth_weight_ * (y_next + y_prev - 2.0 * y_i);

      new_path[i].x = y_i;
      change += std::fabs(y_i - y_i_old);

      double x_i_y = plan[i].y;
      double y_i_y = new_path[i].y;
      double y_prev_y = new_path[i - 1].y;
      double y_next_y = new_path[i + 1].y;

      double y_i_old_y = y_i_y;

      y_i_y += data_weight_ * (x_i_y - y_i_y) +
               smooth_weight_ * (y_next_y + y_prev_y - 2.0 * y_i_y);

      new_path[i].y = y_i_y;
      change += std::fabs(y_i_y - y_i_old_y);
    }

    if (change < tolerance)
      break;

    last_path = new_path;
  }

  return new_path;
}

} // namespace planner
