#pragma once

#include "geometry/pose.h"
#include "grid/global_costmap.h"

namespace geometry {

class CollisionChecker {
public:
  CollisionChecker();

  void setParameters(const grid::GlobalCostmap *costmap, double robot_height,
                     double robot_width);
  void setParameters(const grid::GlobalCostmap *costmap, double robot_radius);
  bool inCollision(Pose2d robot_pose) const;

private:
  double robot_height_, robot_width_;
  double robot_height_pixels_, robot_width_pixels_;

  const grid::GlobalCostmap *global_costmap_;
};

} // namespace geometry
