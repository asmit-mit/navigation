#pragma once

#include <cstddef>
#include <utility>

namespace geometry {

struct State3d {
  size_t x;
  size_t y;
  size_t theta_bin;

  State3d();
  State3d(size_t x, size_t y, size_t theta_bin);
  bool operator==(const State3d &other) const;
};

struct State2d {
  size_t x, y;

  State2d(size_t x, size_t y);
  State2d(const std::pair<size_t, size_t> &s);
  State2d(const State3d &s);
};

struct StateHash {
  std::size_t operator()(const State3d &k) const;
};

}; // namespace geometry
