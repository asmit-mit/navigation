#pragma once

namespace planner {

struct Pose {
  double x;
  double y;
  double theta;

  Pose();
  Pose(double x, double y, double theta);
};

}; // namespace planner
