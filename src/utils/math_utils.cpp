#include "utils/math_utils.h"

namespace utils {

double distance(const geometry::Pose3d &a, const geometry::Pose3d &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double distance(const geometry::Pose2d &a, const geometry::Pose2d &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

geometry::Pose2d perp(const geometry::Pose2d &a, const geometry::Pose2d &b) {
  double mod_b2 = b.norm2();

  if (mod_b2 <= 1e-6)
    return geometry::Pose2d(0, 0);

  double scale = dot(a, b) / mod_b2;
  geometry::Pose2d projection = b * scale;

  return a - projection;
}

double M(double theta) {
  const double two_pi = 2 * M_PI;
  theta = theta - two_pi * std::floor((theta + M_PI) / two_pi);
  return theta;
}

std::pair<double, double> R(double x, double y) {
  double r = std::hypot(x, y);
  double theta = std::atan2(y, x);
  return {r, theta};
}

geometry::Pose3d changeOfBasis(const geometry::Pose3d &p1,
                              const geometry::Pose3d &p2) {
  double theta1 = p1.theta;
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  double new_x = dx * std::cos(theta1) + dy * std::sin(theta1);
  double new_y = -dx * std::sin(theta1) + dy * std::cos(theta1);
  double new_theta = M(p2.theta - p1.theta);

  return geometry::Pose3d(new_x, new_y, new_theta);
}

double M2pi(double theta) {
  const double two_pi = 2 * M_PI;
  return theta - two_pi * std::floor(theta / two_pi);
}

}; // namespace utils
