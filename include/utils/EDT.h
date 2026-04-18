#pragma once

#include <limits>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <vector>

namespace utils {

class EDT {
public:
  EDT();

  void computeDT(nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  double getDistanceAt(size_t idx) const;

public:
  static constexpr double INF = std::numeric_limits<double>::infinity();

private:
  void computeDT1D(size_t start, size_t size, size_t stride);

private:
  std::vector<double> distance_transform_;
};

} // namespace utils
