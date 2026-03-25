#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "planner/pose.h"
#include "planner/state.h"

namespace planner {

class GridMap {
public:
  GridMap();

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  int getDataAt(int x, int y);
  int getDataAt(int idx);
  int getHeight();
  int getWidth();
  double getResolution();

  bool isValid(int x, int y);
  int getIndex(int x, int y);

  std::pair<int, int> worldToMapDiscrete(double x, double y);
  std::pair<double, double> worldToMapContinous(double x, double y);
  std::pair<double, double> mapToWorld(double x, double y);

  State2d pose2dToState2d(const Pose2d &p);
  Pose2d state2dToPose2d(const State2d &s);

private:
  double resolution_;
  int height_, width_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
};

}; // namespace planner
