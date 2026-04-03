#pragma once

#include "planner/pose.h"

#include <vector>

namespace planner {

class Optimizer {
public:
  Optimizer();

  void setWeights(double smooth, double data);
  void setIterations(int iterations);

  std::vector<Pose2d> getSmoothPath(std::vector<Pose2d> &plan);

private:
  int iterations_;
  double smooth_weight_, data_weight_;
};

}; // namespace planner
