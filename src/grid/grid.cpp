#include "grid/grid.h"

#include <cmath>

namespace grid {

Grid::Grid() {
  resolution_ = 0.1;
  height_ = 100;
  width_ = 100;
  origin_x_ = 0.0;
  origin_y_ = 0.0;
}

void Grid::setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
  grid_ = grid;
  height_ = grid->info.height;
  width_ = grid->info.width;
  resolution_ = grid->info.resolution;
  origin_x_ = grid->info.origin.position.x;
  origin_y_ = grid->info.origin.position.y;
}

int Grid::getDataAt(int idx) const { return grid_->data[idx]; }
int Grid::getDataAt(int x, int y) const { return grid_->data[getIndex(x, y)]; }

size_t Grid::getHeight() const { return height_; }
size_t Grid::getWidth() const { return width_; }
double Grid::getResolution() const { return resolution_; }
double Grid::getOriginX() const { return origin_x_; }
double Grid::getOriginY() const { return origin_y_; }

bool Grid::isValid(size_t x, size_t y) const {
  return x < width_ && y < height_ && grid_->data[getIndex(x, y)] != 100;
}

size_t Grid::getIndex(size_t x, size_t y) const { return y * width_ + x; }

}; // namespace grid
