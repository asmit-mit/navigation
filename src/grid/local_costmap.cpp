#include "grid/local_costmap.h"
#include "utils/EDT.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <vector>

namespace grid {

LocalCostmap::LocalCostmap() {
  resolution_ = 0.1;
  height_ = 100;
  width_ = 100;
  origin_x_ = 0.0;
  origin_y_ = 0.0;
  inflation_radius_ = 0.2;
  inscribed_radius_ = 0.1;
  scaling_factor_ = 5.0;
  window_width_ = 0;
  window_height_ = 0;
  window_origin_x_ = 0.0;
  window_origin_y_ = 0.0;
  global_offset_x_ = 0;
  global_offset_y_ = 0;
}

void LocalCostmap::setParameters(nav_msgs::msg::OccupancyGrid::SharedPtr grid,
                                 const utils::EDT *edt, double robot_x,
                                 double robot_y, double window_size,
                                 double inflation_radius,
                                 double inscribed_radius,
                                 double scaling_factor) {
  assert(grid != nullptr);
  assert(edt != nullptr);
  assert(inflation_radius > epsilon_);
  assert(inscribed_radius > epsilon_);
  assert(scaling_factor > epsilon_);
  assert(window_size > epsilon_);

  edt_ = edt;
  grid_ = grid;
  height_ = grid->info.height;
  width_ = grid->info.width;
  resolution_ = grid->info.resolution;
  origin_x_ = grid->info.origin.position.x;
  origin_y_ = grid->info.origin.position.y;

  inflation_radius_ = inflation_radius;
  inscribed_radius_ = inscribed_radius;
  scaling_factor_ = scaling_factor;

  auto [center_cell_x, center_cell_y] = worldToMapDiscrete(robot_x, robot_y);

  size_t half_cells =
      static_cast<int>(std::ceil((window_size / 2.0) / resolution_));

  size_t x_min = std::max(0UL, center_cell_x - half_cells);
  size_t y_min = std::max(0UL, center_cell_y - half_cells);
  size_t x_max = std::min(width_ - 1, center_cell_x + half_cells);
  size_t y_max = std::min(height_ - 1, center_cell_y + half_cells);

  window_width_ = x_max - x_min + 1;
  window_height_ = y_max - y_min + 1;

  global_offset_x_ = x_min;
  global_offset_y_ = y_min;

  window_origin_x_ = origin_x_ + x_min * resolution_;
  window_origin_y_ = origin_y_ + y_min * resolution_;

  costmap_.resize(window_width_ * window_height_);

  computeDistToCostMap();
  computeCostmap();
}

double LocalCostmap::getCostAt(size_t idx) const { return costmap_[idx]; }

double LocalCostmap::getCostAt(size_t local_x, size_t local_y) const {
  return costmap_[localIndex(local_x, local_y)];
}

double LocalCostmap::getCostAtWorld(double world_x, double world_y) const {
  auto [local_x, local_y] = worldToMapDiscrete(world_x, world_y);

  if (local_x >= window_width_ || local_y >= window_height_)
    return -1.0;

  return getCostAt(local_x, local_y);
}

double LocalCostmap::getDistanceAt(int idx) const {
  return edt_->getDistanceAt(idx);
}

double LocalCostmap::getDistanceAt(size_t local_x, size_t local_y) const {
  return edt_->getDistanceAt(localIndex(local_x, local_y));
}

double LocalCostmap::getDistanceAtWorld(double world_x, double world_y) const {
  auto [local_x, local_y] = worldToMapDiscrete(world_x, world_y);

  if (local_x >= window_width_ || local_y >= window_height_)
    return -1.0;

  return getDistanceAt(local_x, local_y);
}

size_t LocalCostmap::getWindowWidth() const { return window_width_; }
size_t LocalCostmap::getWindowHeight() const { return window_height_; }
double LocalCostmap::getWindowOriginX() const { return window_origin_x_; }
double LocalCostmap::getWindowOriginY() const { return window_origin_y_; }

void LocalCostmap::computeCostmap() {
  size_t size = window_width_ * window_height_;
  for (size_t i = 0; i < size; i++) {
    size_t global_x = global_offset_x_ + (i % window_width_);
    size_t global_y = global_offset_y_ + (i / window_width_);
    size_t global_idx = global_y * width_ + global_x;

    double dist = edt_->getDistanceAt(global_idx);
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

double LocalCostmap::computeCost(double dist) {
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

void LocalCostmap::computeDistToCostMap() {
  const size_t max_grid_dist_scaled =
      static_cast<size_t>((inflation_radius_ / resolution_) * COST_PRECISION) +
      1;
  dist_to_cost_.resize(max_grid_dist_scaled + 1);

  for (size_t i = 0; i <= max_grid_dist_scaled; i++) {
    double dist = static_cast<double>(i) / COST_PRECISION;
    dist_to_cost_[i] = computeCost(dist);
  }
}

size_t LocalCostmap::localIndex(size_t local_x, size_t local_y) const {
  return local_y * window_width_ + local_x;
}

} // namespace grid
