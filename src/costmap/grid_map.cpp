#include "costmap/grid_map.h"

#include <cmath>

namespace costmap {

GridMap::GridMap() {
  resolution_ = 0.1;
  height_ = 100;
  width_ = 100;
}

void GridMap::setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
  grid_ = grid;
  height_ = grid->info.height;
  width_ = grid->info.width;
  resolution_ = grid->info.resolution;
}

int GridMap::getDataAt(int idx) const { return grid_->data[idx]; }
int GridMap::getDataAt(int x, int y) const {
  return grid_->data[getIndex(x, y)];
}

int GridMap::getHeight() const { return height_; }
int GridMap::getWidth() const { return width_; }
double GridMap::getResolution() const { return resolution_; }

bool GridMap::isValid(int x, int y) const {
  return x >= 0 && y >= 0 && x < width_ && y < height_ &&
         grid_->data[getIndex(x, y)] != 100;
}

int GridMap::getIndex(int x, int y) const { return y * width_ + x; }

std::pair<int, int> GridMap::worldToMapDiscrete(double x, double y) const {
  int gx = floor((x - grid_->info.origin.position.x) / resolution_);
  int gy = floor((y - grid_->info.origin.position.y) / resolution_);
  return {gx, gy};
}

std::pair<double, double> GridMap::worldToMapContinous(double x,
                                                       double y) const {
  double gx = (x - grid_->info.origin.position.x) / resolution_;
  double gy = (y - grid_->info.origin.position.y) / resolution_;
  return {gx, gy};
}

std::pair<double, double> GridMap::mapToWorld(double x, double y) const {
  double wx = x * resolution_ + grid_->info.origin.position.x;
  double wy = y * resolution_ + grid_->info.origin.position.y;
  return {wx, wy};
}

}; // namespace planner
