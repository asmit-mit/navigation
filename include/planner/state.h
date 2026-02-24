#pragma once

#include <utility>
namespace planner {

struct State3d {
  int grid_x;
  int grid_y;
  int theta_bin;

  State3d();
  State3d(int grid_x, int grid_y, int theta_bin);
  bool operator==(const State3d &other) const;
};


struct State2d {
  int x, y;

  State2d(const std::pair<int, int> &s);
  State2d(const State3d &s);
};

struct StateHash {
  std::size_t operator()(const State3d &k) const;
};

}; // namespace planner
