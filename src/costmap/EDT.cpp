#include "costmap/EDT.h"

#include <climits>
#include <cmath>

namespace costmap {

void EDT::setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
  grid_.setGrid(grid);
  height_ = grid_.getHeight();
  width_ = grid_.getWidth();

  distance_transform_.assign(height_ * width_, INT_MAX);
  for (int i = 0; i < width_ * height_; i++) {
    if (grid_.getDataAt(i) == 100)
      distance_transform_[i] = 0;
    else
      distance_transform_[i] = INT_MAX;
  }
}

int EDT::distanceToObstacle(int x, int y) {
  return distance_transform_[grid_.getIndex(x, y)];
}

void EDT::computeDT() {
  for (int y = 0; y < height_; y++) {
    int start = y * width_;
    computeDT1D(start, width_, 1);
  }

  for (int x = 0; x < width_; x++) {
    int start = x;
    computeDT1D(start, height_, width_);
  }
}

void EDT::computeDT1D(int start, int size, int stride) {
  for (int i = 1; i < size; i++) {
    int curr = start + i * stride;
    int prev = start + (i - 1) * stride;

    if (distance_transform_[prev] != INT_MAX) {
      distance_transform_[curr] =
          std::min(distance_transform_[curr], distance_transform_[prev] + 1);
    }
  }

  for (int i = size - 2; i >= 0; i--) {
    int curr = start + i * stride;
    int next = start + (i + 1) * stride;

    if (distance_transform_[next] != INT_MAX) {
      distance_transform_[curr] =
          std::min(distance_transform_[curr], distance_transform_[next] + 1);
    }
  }
}

}; // namespace costmap
