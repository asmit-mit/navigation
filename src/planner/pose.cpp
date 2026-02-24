#include "planner/pose.h"

namespace planner {

Pose2d::Pose2d(const std::pair<double, double> &p) {
  x = p.first;
  y = p.second;
}

Pose2d::Pose2d(const Pose3d &p) {
  x = p.x;
  y = p.y;
}

Pose3d::Pose3d() {
  x = -1;
  y = -1;
  theta = -1;
}

Pose3d::Pose3d(double x, double y, double theta) : x(x), y(y), theta(theta) {}

}; // namespace planner
