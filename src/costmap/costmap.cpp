#include "costmap/costmap.h"

#include <assert.h>
#include <cmath>
#include <limits>
#include <vector>

namespace costmap {

Costmap::Costmap() {
  inflation_radius_ = 0.2;
  inscribed_radius_ = 0.1;
  scaling_factor_ = 5.0;
};

void Costmap::setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
  grid_ = grid;
  height_ = grid->info.height;
  width_ = grid->info.width;
  resolution_ = grid->info.resolution;
  origin_x_ = grid->info.origin.position.x;
  origin_y_ = grid->info.origin.position.y;

  computeDT();
}

void Costmap::setParameters(double radius, double scaling_factor) {
  assert(radius > epsilon_);
  assert(scaling_factor > epsilon_);

  inflation_radius_ = radius;
  inscribed_radius_ = 0.1;

  scaling_factor_ = scaling_factor;

  computeDistToCostMap();
}

double Costmap::getCostAt(int idx) const { return costmap_[idx]; }

double Costmap::getCostAt(int x, int y) const {
  return costmap_[getIndex(x, y)];
}

void Costmap::computeCostmap() {
  int size = height_ * width_;

  for (int i = 0; i < size; i++) {
    double dist_sq = costmap_[i];
    double dist = std::sqrt(dist_sq);

    int idx = static_cast<int>(dist * COST_PRECISION + 0.5);

    if (idx >= static_cast<int>(dist_to_cost_.size())) {
      costmap_[i] = 0.0;
      continue;
    }

    costmap_[i] = dist_to_cost_[idx];
  }
}

double Costmap::computeCost(double dist) {
  if (dist <= epsilon_) {
    return LETHAL_COST;
  } else if (dist * resolution_ <= inscribed_radius_) {
    return INSCRIBED_COST;
  } else {
    double factor =
        exp(-1.0 * scaling_factor_ * (dist * resolution_ - inscribed_radius_));
    return (INSCRIBED_COST - 1) * factor;
  }
}

void Costmap::computeDistToCostMap() {
  const int max_grid_dist_scaled_ =
      (inflation_radius_ / resolution_) * COST_PRECISION + 1;
  dist_to_cost_.resize(max_grid_dist_scaled_ + 1);

  for (int i = 0; i <= max_grid_dist_scaled_; i++) {
    double dist = static_cast<double>(i) / COST_PRECISION;
    dist_to_cost_[i] = computeCost(dist);
  }
}

void Costmap::computeDT() {
  int size = height_ * width_;
  costmap_.resize(size);

  for (int i = 0; i < size; i++)
    costmap_[i] = (getDataAt(i) == 100) ? 0 : INF;

  for (int y = 0; y < height_; y++)
    computeDT1D(y * width_, width_, 1);

  for (int x = 0; x < width_; x++)
    computeDT1D(x, height_, width_);
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
