#pragma once

#include <cstddef>
namespace planner {

struct State {
  int grid_x;
  int grid_y;
  int theta_bin;

  State(int grid_x, int grid_y, int theta_bin);
  bool operator==(const State &other) const;
};

struct StateHash {
  std::size_t operator()(const State &k) const;
};

}; // namespace planner
