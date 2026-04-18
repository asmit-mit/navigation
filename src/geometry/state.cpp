#include "geometry/state.h"

#include <functional>

namespace geometry {

State3d::State3d() {}

State3d::State3d(size_t grid_x, size_t grid_y, size_t theta_bin)
    : x(grid_x), y(grid_y), theta_bin(theta_bin) {}

bool State3d::operator==(const State3d &other) const {
  return x == other.x && y == other.y && theta_bin == other.theta_bin;
}

State2d::State2d(size_t x, size_t y) : x(x), y(y) {}

State2d::State2d(const std::pair<size_t, size_t> &s) {
  x = s.first;
  y = s.second;
}

State2d::State2d(const State3d &s) {
  x = s.x;
  y = s.y;
}

std::size_t StateHash::operator()(const State3d &k) const {
  std::size_t seed = 0;
  seed ^= std::hash<size_t>{}(k.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<size_t>{}(k.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^=
      std::hash<size_t>{}(k.theta_bin) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

}; // namespace planner
