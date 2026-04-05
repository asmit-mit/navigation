#include "planner/motion_model.h"

namespace planner {

MotionModel::MotionModel() {}

void MotionModel::setMotionModel(MotionModelType type) { type_ = type; }

void MotionModel::setDistanceResolution(double resolution) {
  dubins_.setDistanceResolution(resolution);
  reed_shepps_.setDistanceResolution(resolution);
}

void MotionModel::setTolerance(double angle, double distance) {
  dubins_.setTolerance(angle, distance);
  reed_shepps_.setTolerance(angle, distance);
}

void MotionModel::setMinTurningRadius(double min_radius) {
  dubins_.setMinTurningRadius(min_radius);
  reed_shepps_.setMinTurningRadius(min_radius);
}

void MotionModel::simulate(const Pose3d &start, const Pose3d &end) {
  if (type_ == MotionModelType::DUBINS) {
    dubins_.simulate(start, end);
  } else {
    reed_shepps_.simulate(start, end);
  }
}

MotionModelType MotionModel::getType() const { return type_; }

double MotionModel::getOptimalDistance() {
  if (type_ == MotionModelType::DUBINS) {
    return dubins_.getOptimalDistance();
  } else {
    return reed_shepps_.getOptimalDistance();
  }
}

std::vector<Pose2d> MotionModel::getOptimalPath() {
  if (type_ == MotionModelType::DUBINS) {
    return dubins_.getOptimalPath();
  } else {
    return reed_shepps_.getOptimalPath();
  }
}

} // namespace planner
