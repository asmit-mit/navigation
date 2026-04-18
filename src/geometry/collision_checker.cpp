#include "geometry/collison_checker.h"
#include "grid/global_costmap.h"

namespace geometry {

CollisionChecker::CollisionChecker() {}

void CollisionChecker::setParameters(const grid::GlobalCostmap *costmap,
                                     double robot_height, double robot_width) {
  global_costmap_ = costmap;

  robot_height_ = robot_height;
  robot_width_ = robot_width;
  robot_height_pixels_ = robot_height_ / costmap->getResolution();
  robot_width_pixels_ = robot_width_ / costmap->getResolution();
}

void CollisionChecker::setParameters(const grid::GlobalCostmap *costmap,
                                     double robot_radius) {
  global_costmap_ = costmap;

  robot_height_ = robot_radius;
  robot_width_ = robot_radius;
  robot_height_pixels_ = robot_height_ / costmap->getResolution();
  robot_width_pixels_ = robot_height_pixels_;
}

bool CollisionChecker::inCollision(Pose2d robot_pose) const {
  auto [rx, ry] =
      global_costmap_->worldToMapDiscrete(robot_pose.x, robot_pose.y);

  int half_w = robot_width_pixels_ / 2;
  int half_h = robot_height_pixels_ / 2;

  int size_x = global_costmap_->getWidth();
  int size_y = global_costmap_->getHeight();

  int min_x = std::max(0, rx - half_w);
  int max_x = std::min(size_x - 1, rx + half_w);
  int min_y = std::max(0, ry - half_h);
  int max_y = std::min(size_y - 1, ry + half_h);

  for (int x = min_x; x <= max_x; x++) {
    for (int y = min_y; y <= max_y; y++) {
      auto cost = global_costmap_->getCostAt(x, y);

      if (cost == grid::GlobalCostmap::LETHAL_COST ||
          cost == grid::GlobalCostmap::INSCRIBED_COST) {
        return true;
      }
    }
  }

  return false;
}

} // namespace geometry
