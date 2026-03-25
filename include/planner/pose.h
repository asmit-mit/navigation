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

  Pose2d();
  Pose2d(const std::pair<double, double> &p);
  Pose2d(const Pose3d &p);
  Pose2d(double x, double y);

  Pose2d operator+(const Pose2d &other) const;
  Pose2d operator-(const Pose2d &other) const;
  Pose2d operator*(double scalar) const;
  Pose2d operator/(double scalar) const;

  Pose2d normalized() const;
  double norm() const;
  double norm2() const;
};

}; // namespace planner
