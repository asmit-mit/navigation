#pragma once

#include <cmath>
#include <utility>

#include "planner/pose.h"

namespace utils {

double distance(const planner::Pose3d &a, const planner::Pose3d &b);
double distance(const planner::Pose2d &a, const planner::Pose2d &b);

double dot(const planner::Pose2d &a, const planner::Pose2d &b);
planner::Pose2d perp(const planner::Pose2d &a, const planner::Pose2d &b);

double M(double theta);
std::pair<double, double> R(double x, double y);
planner::Pose3d changeOfBasis(const planner::Pose3d &p1,
                              const planner::Pose3d &p2);
double rad2deg(double rad);
double deg2rad(double deg);
int sign(int x);

}; // namespace utils
