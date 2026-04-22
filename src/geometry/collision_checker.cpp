#include "geometry/collison_checker.h"
#include "utils/grid_utils.h"

#include <cmath>
#include <iostream>

namespace geometry {

CollisionChecker::CollisionChecker() {}

void CollisionChecker::setParameters(const grid::GlobalCostmap *global_costmap,
                                     const grid::LocalCostmap *local_costmap,
                                     double robot_height,
                                     double robot_width) {
  global_costmap_ = global_costmap;
  local_costmap_ = local_costmap;

  robot_height_ = robot_height;
  robot_width_ = robot_width;
  robot_height_pixels_ = robot_height_ / global_costmap->getResolution();
  robot_width_pixels_ = robot_width_ / global_costmap->getResolution();
}

void CollisionChecker::setParameters(const grid::GlobalCostmap *global_costmap,
                                     const grid::LocalCostmap *local_costmap,
                                     double robot_radius) {
  global_costmap_ = global_costmap;
  local_costmap_ = local_costmap;

  robot_height_ = 2 * robot_radius;
  robot_width_ = 2 * robot_radius;
  robot_height_pixels_ = robot_height_ / global_costmap->getResolution();
  robot_width_pixels_ = robot_height_pixels_;
}

bool CollisionChecker::inCollisionGlobal(const Pose2d &robot_pose) const {
  auto [rx, ry] = utils::worldToMapDiscrete(robot_pose.x,
                                            robot_pose.y,
                                            global_costmap_->getOriginX(),
                                            global_costmap_->getOriginY(),
                                            global_costmap_->getResolution());

  int half_w = robot_width_pixels_ / 2;
  int half_h = robot_height_pixels_ / 2;

  size_t size_x = global_costmap_->getWidth();
  size_t size_y = global_costmap_->getHeight();

  size_t min_x = std::max(0, rx - half_w);
  size_t max_x = std::min(size_x - 1, (size_t)rx + half_w);
  size_t min_y = std::max(0, ry - half_h);
  size_t max_y = std::min(size_y - 1, (size_t)ry + half_h);

  for (size_t x = min_x; x <= max_x; x++) {
    for (size_t y = min_y; y <= max_y; y++) {
      auto cost = global_costmap_->getCostAt(x, y);

      if (cost >= grid::GlobalCostmap::INSCRIBED_COST)
        return true;
    }
  }

  return false;
}

bool CollisionChecker::inCollisionLocal(const Pose2d &robot_pose) const {
  auto [rx, ry] = utils::worldToMapDiscrete(robot_pose.x,
                                            robot_pose.y,
                                            local_costmap_->getWindowOriginX(),
                                            local_costmap_->getWindowOriginY(),
                                            global_costmap_->getResolution());

  int half_w = robot_width_pixels_ / 2;
  int half_h = robot_height_pixels_ / 2;

  size_t size_x = local_costmap_->getWindowWidth();
  size_t size_y = local_costmap_->getWindowHeight();

  size_t min_x = std::max(0, rx - half_w);
  size_t max_x = std::min(size_x - 1, (size_t)rx + half_w);
  size_t min_y = std::max(0, ry - half_h);
  size_t max_y = std::min(size_y - 1, (size_t)ry + half_h);

  for (size_t x = min_x; x <= max_x; x++) {
    for (size_t y = min_y; y <= max_y; y++) {
      auto cost = local_costmap_->getCostAt(x, y);

      if (cost >= grid::LocalCostmap::INSCRIBED_COST)
        return true;
    }
  }

  return false;
}

} // namespace geometry
