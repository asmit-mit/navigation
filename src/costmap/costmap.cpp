#include "costmap/costmap.h"
#include "planner/grid_map.h"

#include <cmath>
#include <limits>
#include <vector>

namespace costmap {

void Costmap::setGrid(const planner::GridMap *grid) {
  grid_ = grid;

  int size = grid_->getHeight() * grid_->getWidth();
  costmap_.resize(size);

  for (int i = 0; i < size; i++) {
    if (grid_->getDataAt(i) == 100) {
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

void Costmap::computeCostmap() {
  int height = grid_->getHeight();
  int width = grid_->getWidth();
  double resolution = grid_->getResolution();

  for (int y = 0; y < height; y++)
    computeDT1D(y * width, width, 1);

  for (int x = 0; x < width; x++)
    computeDT1D(x, height, width);

  for (int i = 0; i < height * width; i++) {
    double sq_dist = costmap_[i];

    if (sq_dist < 1e-12) {
      costmap_[i] = 254;
    } else if (sq_dist <= radius_ * radius_) {
      costmap_[i] = 253;
    } else {
      double dist = std::sqrt(sq_dist);
      dist *= resolution;

      double cost = 252.0 * std::exp(-scaling_factor_ * (dist - radius_));
      cost = std::max(0.0, std::min(252.0, cost));
      costmap_[i] = cost;
    }
  }
}

double Costmap::getDataAt(int idx) const { return costmap_[idx]; }

double Costmap::getDataAt(int x, int y) const {
  return costmap_[grid_->getIndex(x, y)];
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
