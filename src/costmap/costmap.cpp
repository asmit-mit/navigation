#include "costmap/costmap.h"

#include <cmath>
#include <limits>
#include <vector>

namespace costmap {

void Costmap::setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
  grid_ = grid;
  height_ = grid->info.height;
  width_ = grid->info.width;
  resolution_ = grid->info.resolution;

  int size = height_ * width_;
  costmap_.resize(size);
  costmap_.resize(size);

  for (int i = 0; i < size; i++) {
    if (getDataAt(i) == 100) {
      costmap_[i] = 0;
    } else {
      costmap_[i] = INF;
    }
  }
}

void Costmap::setRadius(double radius) { radius_ = radius; }

void Costmap::setScalingFactor(double scaling_factor) {
  scaling_factor_ = scaling_factor;
}

void Costmap::setAllowUnknown(bool allow_unknown) {
  allow_unknown_ = allow_unknown;
}

void Costmap::computeCostmap() {
  for (int y = 0; y < height_; y++)
    computeDT1D(y * width_, width_, 1);

  for (int x = 0; x < width_; x++)
    computeDT1D(x, height_, width_);

  for (int i = 0; i < height_ * width_; i++) {
    double sq_dist_cells = costmap_[i];
    double dist = std::sqrt(sq_dist_cells) * resolution_;

    double raw_cost;
    if (dist < 1e-6) {
      raw_cost = 254.0;
    } else if (dist <= radius_) {
      raw_cost = 253.0;
    } else {
      raw_cost = 252.0 * std::exp(-scaling_factor_ * (dist - radius_));

      raw_cost = std::clamp(raw_cost, 0.0, 252.0);
    }

    costmap_[i] = raw_cost;
  }
}

double Costmap::getCostAt(int idx) const { return costmap_[idx]; }

double Costmap::getCostAt(int x, int y) const {
  return costmap_[getIndex(x, y)];
}

void Costmap::computeDT1D(int start, int size, int stride) {
  std::vector<int> v(size);
  std::vector<double> z(size + 1);

  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;

  auto f = [&](int q) -> double { return costmap_[start + q * stride]; };

  for (int q = 1; q < size; q++) {
    double s;

    while (true) {
      int vk = v[k];

      s = ((f(q) + q * q) - (f(vk) + vk * vk)) / (2.0 * (q - vk));

      if (s > z[k])
        break;

      k--;
      if (k < 0) {
        k = 0;
        break;
      }
    }

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }

  k = 0;
  for (int q = 0; q < size; q++) {
    while (z[k + 1] < q) {
      k++;
    }

    int vk = v[k];
    double dx = q - vk;

    costmap_[start + q * stride] = dx * dx + f(vk);
  }
}

} // namespace costmap
