#pragma once

#include <utility>
namespace planner {

struct Pose3d {
  double x;
  double y;
  double theta;

  Pose3d();
  Pose3d(double x, double y, double theta);
};


struct Pose2d {
  double x, y;

  Pose2d(const std::pair<double, double> &p);
  Pose2d(const Pose3d &p);
};

}; // namespace planner
