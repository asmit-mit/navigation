#pragma once

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

} // namespace planner
