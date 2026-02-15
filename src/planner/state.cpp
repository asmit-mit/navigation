#include "planner/state.h"

namespace planner {

State::State(int grid_x, int grid_y, int theta_bin)
    : grid_x(grid_x), grid_y(grid_y), theta_bin(theta_bin) {}

bool State::operator==(const State &other) const {
  return grid_x == other.grid_x && grid_y == other.grid_y &&
         theta_bin == other.theta_bin;
}

std::size_t StateHash::operator()(const State &k) const {
  return ((k.grid_x * 73856093) ^ (k.grid_y * 19349663) ^
          (k.theta_bin * 83492791));
}

}; // namespace planner
