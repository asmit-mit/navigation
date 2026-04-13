#pragma once

#include <cmath>
#include <utility>

#include "geometry/pose.h"

namespace utils {

double distance(const geometry::Pose3d &a, const geometry::Pose3d &b);
double distance(const geometry::Pose2d &a, const geometry::Pose2d &b);

geometry::Pose2d perp(const geometry::Pose2d &a, const geometry::Pose2d &b);

double M(double theta);
std::pair<double, double> R(double x, double y);
geometry::Pose3d changeOfBasis(const geometry::Pose3d &p1,
                              const geometry::Pose3d &p2);
double M2pi(double theta);

inline double dot(const geometry::Pose2d &a, const geometry::Pose2d &b) {
  return a.x * b.x + a.y * b.y;
}

inline double rad2deg(double rad) { return 180 * rad / M_PI; }

inline double deg2rad(double deg) { return M_PI * deg / 180; }

inline int sign(int x) { return (x > 0) - (x < 0); }

}; // namespace utils
