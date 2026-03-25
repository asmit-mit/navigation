#include "planner/pose.h"
#include <cmath>

namespace planner {

Pose2d::Pose2d() {
  x = 0;
  y = 0;
}

Pose2d::Pose2d(const std::pair<double, double> &p) {
  x = p.first;
  y = p.second;
}

Pose2d::Pose2d(const Pose3d &p) {
  x = p.x;
  y = p.y;
}

Pose2d::Pose2d(double x, double y) : x(x), y(y) {}

Pose2d Pose2d::operator+(const Pose2d &other) const {
  return Pose2d(x + other.x, y + other.y);
}

Pose2d Pose2d::operator-(const Pose2d &other) const {
  return Pose2d(x - other.x, y - other.y);
}

Pose2d Pose2d::operator*(double scalar) const {
  return Pose2d(x * scalar, y * scalar);
}

Pose2d Pose2d::operator/(double scalar) const {
  if (scalar <= 1e-8)
    return Pose2d(0.0, 0.0);
  return Pose2d(x / scalar, y / scalar);
}

Pose2d Pose2d::normalized() const {
  double n = norm();
  if (n <= 1e-8)
    return Pose2d(0.0, 0.0);
  return (*this) / n;
}

double Pose2d::norm() const { return std::hypot(x, y); }

double Pose2d::norm2() const { return x * x + y * y; }

Pose3d::Pose3d() {
  x = -1;
  y = -1;
  theta = -1;
}

Pose3d::Pose3d(double x, double y, double theta) : x(x), y(y), theta(theta) {}

}; // namespace planner
