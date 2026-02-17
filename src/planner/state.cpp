#include "planner/state.h"
#include <functional>

namespace planner {

State::State() {}

State::State(int grid_x, int grid_y, int theta_bin)
    : grid_x(grid_x), grid_y(grid_y), theta_bin(theta_bin) {}

bool State::operator==(const State &other) const {
  return grid_x == other.grid_x && grid_y == other.grid_y &&
         theta_bin == other.theta_bin;
}

std::size_t StateHash::operator()(const State &k) const {
  std::size_t seed = 0;
  seed ^= std::hash<int>{}(k.grid_x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>{}(k.grid_y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>{}(k.theta_bin) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

}; // namespace planner
