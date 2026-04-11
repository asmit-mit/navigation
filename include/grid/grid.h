#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace grid {

class Grid {
public:
  Grid();
  virtual ~Grid() = default;

  virtual void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  int getDataAt(int x, int y) const;
  int getDataAt(int idx) const;
  int getHeight() const;
  int getWidth() const;
  double getResolution() const;
  double getOriginX() const;
  double getOriginY() const;

  bool isValid(int x, int y) const;
  int getIndex(int x, int y) const;

  std::pair<int, int> worldToMapDiscrete(double x, double y) const;
  std::pair<double, double> worldToMapContinous(double x, double y) const;
  std::pair<double, double> mapToWorld(double x, double y) const;

protected:
  double resolution_;
  int height_, width_;
  double origin_x_, origin_y_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
};

} // namespace grid
