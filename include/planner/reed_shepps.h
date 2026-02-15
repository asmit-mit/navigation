#include <array>
#include <cmath>
#include <utility>

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

  void setMinTurningRadius(double min_turning_radius_);
  double getOptimalPath(Pose &start, Pose &end);

private:
  double M(double theta);
  std::pair<double, double> R(double x, double y);
  Pose changeOfBasis(Pose &p1, Pose &p2);
  double rad2deg(double rad);
  double deg2rad(double deg);
  int sign(int x);

  double getAllPaths();

  double path1(const Pose &p);
  double path2(const Pose &p);
  double path3(const Pose &p);
  double path4(const Pose &p);
  double path5(const Pose &p);
  double path6(const Pose &p);
  double path7(const Pose &p);
  double path8(const Pose &p);
  double path9(const Pose &p);
  double path10(const Pose &p);
  double path11(const Pose &p);
  double path12(const Pose &p);

private:
  double min_turning_radius_;

  using pathFn = double (ReedShepps::*)(const Pose &);

  const std::array<pathFn, 12> pathFns = {
      &ReedShepps::path1,  &ReedShepps::path2,  &ReedShepps::path3,
      &ReedShepps::path4,  &ReedShepps::path5,  &ReedShepps::path6,
      &ReedShepps::path7,  &ReedShepps::path8,  &ReedShepps::path9,
      &ReedShepps::path10, &ReedShepps::path11, &ReedShepps::path12};
};

}; // namespace planner
