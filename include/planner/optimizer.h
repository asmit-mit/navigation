#pragma once

#include "grid/costmap.h"
#include "planner/pose.h"

#include <vector>

namespace planner {

class Optimizer {
public:
  Optimizer();

  void setCostmap(const grid::Costmap *costmap);
  void setWeights(double smooth, double data);
  void setIterations(int iterations);

  std::vector<Pose2d> getSmoothPath(std::vector<Pose2d> &plan) const;

private:
  int iterations_;
  double smooth_weight_, data_weight_;

  const grid::Costmap *costmap_;

  static constexpr double epsilon_ = 1e-6;
};

}; // namespace planner
