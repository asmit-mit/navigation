#pragma once

#include <cmath>
#include <utility>

#include "planner/pose.h"

namespace utils {

double distance(const planner::Pose3d &a, const planner::Pose3d &b);
double distance(const planner::Pose2d &a, const planner::Pose2d &b);

planner::Pose2d perp(const planner::Pose2d &a, const planner::Pose2d &b);

double M(double theta);
std::pair<double, double> R(double x, double y);
planner::Pose3d changeOfBasis(const planner::Pose3d &p1,
                              const planner::Pose3d &p2);
double M2pi(double theta);

inline double dot(const planner::Pose2d &a, const planner::Pose2d &b) {
  return a.x * b.x + a.y * b.y;
}

inline double rad2deg(double rad) { return 180 * rad / M_PI; }

inline double deg2rad(double deg) { return M_PI * deg / 180; }

inline int sign(int x) { return (x > 0) - (x < 0); }

}; // namespace utils
