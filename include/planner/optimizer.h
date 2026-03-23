#pragma once

#include "planner/pose.h"

#include <vector>

namespace planner {

class Optimizer {
public:
  Optimizer();

  void setIterations(int iterations);
  void setStepSize(double step_size);

  std::vector<Pose2d> getSmoothPath(std::vector<Pose2d> &plan);

private:
  struct Grad {
    double x, y;

    Grad();
    Grad(double x, double y);

    Grad operator*(double scalar) const;
    Grad operator+(const Grad &other) const;
  };

private:
  int iterations_;
  double step_;

  std::vector<Grad> grad_;
};

}; // namespace planner
