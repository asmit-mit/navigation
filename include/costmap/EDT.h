#pragma once

#include "planner/grid_map.h"

#include <vector>

namespace costmap {

class EDT {
public:
  EDT() {}

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);
  int distanceToObstacle(int x, int y);
  void computeDT();

private:
  void computeDT1D(int start, int size, int stride);

private:
  int height_, width_;
  planner::GridMap grid_;

  std::vector<int> distance_transform_;
};

} // namespace costmap
