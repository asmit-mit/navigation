#include "planner/state.h"
#include <functional>

namespace planner {

State3d::State3d() {}

State3d::State3d(int grid_x, int grid_y, int theta_bin)
    : x(grid_x), y(grid_y), theta_bin(theta_bin) {}

bool State3d::operator==(const State3d &other) const {
  return x == other.x && y == other.y && theta_bin == other.theta_bin;
}

State2d::State2d(const std::pair<int, int> &s) {
  x = s.first;
  y = s.second;
}

State2d::State2d(const State3d &s) {
  x = s.x;
  y = s.y;
}

std::size_t StateHash::operator()(const State3d &k) const {
  std::size_t seed = 0;
  seed ^= std::hash<int>{}(k.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^=
      std::hash<int>{}(k.theta_bin) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

}; // namespace planner
