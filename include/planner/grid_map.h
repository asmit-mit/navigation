#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace planner {

class GridMap {
public:
  GridMap();

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  int getHeight();
  int getWidth();
  double getResolution();

  bool isValid(int x, int y);
  int getIndex(int x, int y);

  std::pair<int, int> worldToMapDiscrete(double x, double y);
  std::pair<double, double> worldToMapContinous(double x, double y);
  std::pair<double, double> mapToWorld(double x, double y);

private:
  double resolution_;
  int height_, width_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
};

}; // namespace planner
