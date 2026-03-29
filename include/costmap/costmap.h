#pragma once

#include "costmap/grid_map.h"

#include <limits>
#include <vector>

namespace costmap {

class Costmap : public GridMap {
public:
  Costmap() {}

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) override;

  void setRadius(double radius);
  void setScalingFactor(double scaling_factor);
  void setAllowUnknown(bool allow_unknown);
  void computeCostmap();

  double getCostAt(int idx) const;
  double getCostAt(int x, int y) const;

private:
  void computeDT1D(int start, int size, int stride);

private:
  double radius_, scaling_factor_;
  bool allow_unknown_;

  static constexpr double INF = std::numeric_limits<double>::infinity();

  static constexpr uint8_t COST_UNKNOWN_ROS = 255;
  static constexpr uint8_t COST_OBS_ROS = 253;
  static constexpr uint8_t COST_OBS = 254;

  static constexpr double COST_NEUTRAL = 50.0;
  static constexpr double COST_FACTOR = 0.8;

  std::vector<double> costmap_;
};

} // namespace costmap
