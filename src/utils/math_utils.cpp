#include "utils/math_utils.h"

namespace utils {

double distance(const planner::Pose3d &a, const planner::Pose3d &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double distance(const planner::Pose2d &a, const planner::Pose2d &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double M(double theta) {
  theta = std::fmod(theta, 2 * M_PI);

  if (theta < 0)
    theta += 2 * M_PI;

  if (theta >= M_PI)
    theta -= 2 * M_PI;

  return theta;
}

std::pair<double, double> R(double x, double y) {
  double r = std::hypot(x, y);
  double theta = std::atan2(y, x);
  return {r, theta};
}

planner::Pose3d changeOfBasis(const planner::Pose3d &p1,
                              const planner::Pose3d &p2) {
  double theta1 = p1.theta;
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  double new_x = dx * std::cos(theta1) + dy * std::sin(theta1);
  double new_y = -dx * std::sin(theta1) + dy * std::cos(theta1);
  double new_theta = M(p2.theta - p1.theta);

  return planner::Pose3d(new_x, new_y, new_theta);
}

double rad2deg(double rad) { return 180 * rad / M_PI; }

double deg2rad(double deg) { return M_PI * deg / 180; }

}; // namespace utils
