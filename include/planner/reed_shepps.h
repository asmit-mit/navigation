#pragma once

#include <array>
#include <vector>

#include "planner/motion_states.h"
#include "planner/pose.h"
#include "utils/trig_utils.h"

namespace planner {

class ReedShepps {
public:
  ReedShepps();

  void setTrigTable(const utils::TrigTable *trig_table);
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
    Gear gear;

    void reverseGear() {
      if (gear == Gear::FORWARD)
        gear = Gear::BACKWARD;
      else if (gear == Gear::BACKWARD)
        gear = Gear::FORWARD;
    }

    void reverseSteering() {
      if (steering == Steering::LEFT)
        steering = Steering::RIGHT;
      else if (steering == Steering::RIGHT)
        steering = Steering::LEFT;
    }

    PathElement()
        : param(0), steering(Steering::STRAIGHT), gear(Gear::FORWARD) {}
    PathElement(double p, Steering s, Gear g) {
      if (p >= 0) {
        param = p;
        steering = s;
        gear = g;
      } else {
        param = -p;
        steering = s;
        gear = (g == Gear::FORWARD) ? Gear::BACKWARD : Gear::FORWARD;
      }
    }
  };

private:
  void path1(const Pose3d &p, bool timeflip, bool reflect);
  void path2(const Pose3d &p, bool timeflip, bool reflect);
  void path3(const Pose3d &p, bool timeflip, bool reflect);
  void path4(const Pose3d &p, bool timeflip, bool reflect);
  void path5(const Pose3d &p, bool timeflip, bool reflect);
  void path6(const Pose3d &p, bool timeflip, bool reflect);
  void path7(const Pose3d &p, bool timeflip, bool reflect);
  void path8(const Pose3d &p, bool timeflip, bool reflect);
  void path9(const Pose3d &p, bool timeflip, bool reflect);
  void path10(const Pose3d &p, bool timeflip, bool reflect);
  void path11(const Pose3d &p, bool timeflip, bool reflect);
  void path12(const Pose3d &p, bool timeflip, bool reflect);

  void tryPath(double dist, int count, bool timeflip, bool reflect);

private:
  const utils::TrigTable *trig_table_;

  double min_turning_radius_;
  double angular_tolerance_, distance_tolerance_;
  double distance_resolution_;

  double optimal_path_dist_;
  int optimal_path_segment_count_;

  std::array<PathElement, 5> optimal_path_;
  std::array<PathElement, 5> candidate_path_;

  Pose3d start_, end_;

  using pathFn = void (ReedShepps::*)(const Pose3d &, bool, bool);

  const std::array<pathFn, 12> pathFns = {
      &ReedShepps::path1,  &ReedShepps::path2,  &ReedShepps::path3,
      &ReedShepps::path4,  &ReedShepps::path5,  &ReedShepps::path6,
      &ReedShepps::path7,  &ReedShepps::path8,  &ReedShepps::path9,
      &ReedShepps::path10, &ReedShepps::path11, &ReedShepps::path12};
};

}; // namespace planner
