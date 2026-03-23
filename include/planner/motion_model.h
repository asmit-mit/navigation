#pragma once

#include "planner/dubins.h"
#include "planner/reed_shepps.h"

namespace planner {

enum class MotionModelType {
  DUBINS,
  REED_SHEPPS,
};

class MotionModel {
public:
  MotionModel();

  void setMotionModel(MotionModelType type);
  void setDistanceResolution(double resolution);
  void setTolerance(double angle, double distance);
  void setMinTurningRadius(double min_radius);
  void simulate(const Pose3d &start, const Pose3d &end);
  double getOptimalDistance();
  std::vector<Pose3d> getOptimalPath();

private:
  MotionModelType type_;

  ReedShepps reed_shepps_;
  Dubins dubins_;
};

} // namespace planner
