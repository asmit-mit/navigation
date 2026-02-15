#include "planner/pose.h"

namespace planner {

Pose::Pose() {
  x = -1;
  y = -1;
  theta = -1;
}

Pose::Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}

}; // namespace planner
