#pragma once

#include "planner/grid_map.h"

#include <limits>
#include <vector>

namespace costmap {

class Costmap {
public:
  Costmap() {}

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);
  void setRadius(double radius);
  void setScalingFactor(double scaling_factor);
  void computeCostmap();

  double getDataAt(int x, int y);
  double getDataAt(int idx);

private:
  void computeDT1D(int start, int size, int stride);

private:
  double radius_, scaling_factor_;
  int height_, width_;
  double resolution_;
  planner::GridMap grid_;

  static constexpr double INF = std::numeric_limits<double>::infinity();

  std::vector<double> costmap_;
};

} // namespace costmap
