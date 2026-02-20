#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include "planner/pose.h"

namespace planner {

enum class Steering {
  LEFT = -1,
  RIGHT = 1,
  STRAIGHT = 0,
};

enum class Gear {
  FORWARD = 1,
  BACKWARD = -1,
};

class ReedShepps {
public:
  ReedShepps();

  void setDistanceResolution(double resolution);
  void setMinTurningRadius(double min_radius);
  void simulate(const Pose &start, const Pose &end);
  double getOptimalDistance();
  std::vector<Pose> getOptimalPath();

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
    PathElement(double param, Steering steering, Gear gear)
        : param(param), steering(steering), gear(gear) {}
  };

private:
  double M(double theta);
  std::pair<double, double> R(double x, double y);
  Pose changeOfBasis(const Pose &p1, const Pose &p2);
  double rad2deg(double rad);
  double deg2rad(double deg);
  int sign(int x);

  void path1(const Pose &p, bool timeflip, bool reflect);
  void path2(const Pose &p, bool timeflip, bool reflect);
  void path3(const Pose &p, bool timeflip, bool reflect);
  void path4(const Pose &p, bool timeflip, bool reflect);
  void path5(const Pose &p, bool timeflip, bool reflect);
  void path6(const Pose &p, bool timeflip, bool reflect);
  void path7(const Pose &p, bool timeflip, bool reflect);
  void path8(const Pose &p, bool timeflip, bool reflect);
  void path9(const Pose &p, bool timeflip, bool reflect);
  void path10(const Pose &p, bool timeflip, bool reflect);
  void path11(const Pose &p, bool timeflip, bool reflect);
  void path12(const Pose &p, bool timeflip, bool reflect);

private:
  double min_turning_radius_;
  double optimal_path_dist_;
  double distance_resolution_;
  std::vector<PathElement> optimal_path_;

  Pose start_, end_;

  using pathFn = void (ReedShepps::*)(const Pose &, bool, bool);

  const std::array<pathFn, 12> pathFns = {
      &ReedShepps::path1,  &ReedShepps::path2,  &ReedShepps::path3,
      &ReedShepps::path4,  &ReedShepps::path5,  &ReedShepps::path6,
      &ReedShepps::path7,  &ReedShepps::path8,  &ReedShepps::path9,
      &ReedShepps::path10, &ReedShepps::path11, &ReedShepps::path12};
};

}; // namespace planner
