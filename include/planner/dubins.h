#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "planner/motion_states.h"
#include "planner/pose.h"

namespace planner {

class Dubins {
public:
  Dubins();

  void setDistanceResolution(double resolution);
  void setTolerance(double angle, double distance);
  void setMinTurningRadius(double min_radius);
  void simulate(const Pose3d &start, const Pose3d &end);
  double getOptimalDistance();
  std::vector<Pose2d> getOptimalPath();

private:
  struct PathElement {
    double param;
    Steering steering;

    PathElement() : param(0.0), steering(Steering::STRAIGHT) {}

    PathElement(double p, Steering s) : param(std::abs(p)), steering(s) {}

    void reverseSteering() {
      if (steering == Steering::LEFT)
        steering = Steering::RIGHT;
      else if (steering == Steering::RIGHT)
        steering = Steering::LEFT;
    }
  };

private:
  void pathLSL(const Pose3d &p, bool reflect);
  void pathRSR(const Pose3d &p, bool reflect);
  void pathLSR(const Pose3d &p, bool reflect);
  void pathRSL(const Pose3d &p, bool reflect);
  void pathRLR(const Pose3d &p, bool reflect);
  void pathLRL(const Pose3d &p, bool reflect);

  void tryPath(double dist, int count, bool reflect);
  void computeParams(const Pose3d &p, double &alpha, double &beta, double &d);

private:
  double min_turning_radius_;
  double angular_tolerance_, distance_tolerance_;
  double distance_resolution_;

  double optimal_path_dist_;
  int optimal_path_segment_count_;
  std::array<PathElement, 3> optimal_path_;
  std::array<PathElement, 3> candidate_path_;

  Pose3d start_, end_;
};

}; // namespace planner
