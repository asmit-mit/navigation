#pragma once

#include "grid/global_costmap.h"
#include "geometry/pose.h"

#include <vector>

namespace planner {

class Optimizer {
public:
  Optimizer();

  void setCostmap(const grid::GlobalCostmap *costmap);
  void setWeights(double smooth, double data);
  void setIterations(int iterations);

  std::vector<geometry::Pose3d> getSmoothPath(std::vector<geometry::Pose3d> &plan) const;

private:
  int iterations_;
  double smooth_weight_, data_weight_;

  const grid::GlobalCostmap *costmap_;

  static constexpr double epsilon_ = 1e-6;
};

}; // namespace planner
