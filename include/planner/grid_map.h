#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace planner {

class GridMap {
public:
  GridMap();

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  int getDataAt(int x, int y) const;
  int getDataAt(int idx) const;
  int getHeight() const;
  int getWidth() const;
  double getResolution() const;

  bool isValid(int x, int y) const;
  int getIndex(int x, int y) const;

  std::pair<int, int> worldToMapDiscrete(double x, double y) const;
  std::pair<double, double> worldToMapContinous(double x, double y) const;
  std::pair<double, double> mapToWorld(double x, double y) const;

private:
  double resolution_;
  int height_, width_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
};

}; // namespace planner
