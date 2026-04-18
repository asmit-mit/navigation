#include "grid/global_costmap.h"
#include "utils/EDT.h"

#include <assert.h>
#include <cmath>
#include <vector>

namespace grid {

GlobalCostmap::GlobalCostmap() {
  inflation_radius_ = 0.2;
  inscribed_radius_ = 0.1;
  scaling_factor_ = 5.0;
  width_ = 0;
  height_ = 0;
  origin_x_ = 0.0;
  origin_y_ = 0.0;
  resolution_ = 0.1;
};

void GlobalCostmap::setParameters(nav_msgs::msg::OccupancyGrid::SharedPtr grid,
                                  const utils::EDT *edt,
                                  double inflation_radius,
                                  double inscribed_radius,
                                  double scaling_factor) {
  assert(grid != nullptr);
  assert(edt != nullptr);
  assert(inflation_radius > epsilon_);
  assert(inscribed_radius > epsilon_);
  assert(scaling_factor > epsilon_);

  edt_ = edt;
  grid_ = grid;
  height_ = grid->info.height;
  width_ = grid->info.width;
  resolution_ = grid->info.resolution;
  origin_x_ = grid->info.origin.position.x;
  origin_y_ = grid->info.origin.position.y;

  costmap_.resize(height_ * width_, 0.0);

  inflation_radius_ = inflation_radius;
  inscribed_radius_ = inscribed_radius;

  scaling_factor_ = scaling_factor;

  computeDistToCostMap();
  computeCostmap();
}

double GlobalCostmap::getCostAt(size_t idx) const { return costmap_[idx]; }

double GlobalCostmap::getCostAt(size_t x, size_t y) const {
  return costmap_[getIndex(x, y)];
}

void GlobalCostmap::computeCostmap() {
  int size = height_ * width_;

  for (int i = 0; i < size; i++) {
    double dist = edt_->getDistanceAt(i);
    if (!std::isfinite(dist)) {
      costmap_[i] = 0.0;
      continue;
    }

    size_t idx = static_cast<int>(dist * COST_PRECISION + 0.5);

    if (idx >= dist_to_cost_.size()) {
      costmap_[i] = 0.0;
      continue;
    }

    costmap_[i] = dist_to_cost_[idx];
  }
}

double GlobalCostmap::computeCost(double dist) {
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

void GlobalCostmap::computeDistToCostMap() {
  const size_t max_grid_dist_scaled_ =
      (inflation_radius_ / resolution_) * COST_PRECISION + 1;
  dist_to_cost_.resize(max_grid_dist_scaled_ + 1);

  for (size_t i = 0; i <= max_grid_dist_scaled_; i++) {
    double dist = static_cast<double>(i) / COST_PRECISION;
    dist_to_cost_[i] = computeCost(dist);
  }
}

} // namespace grid
