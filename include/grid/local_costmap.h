#pragma once

#include "grid/grid.h"
#include "utils/EDT.h"

#include <vector>

namespace grid {

class LocalCostmap : public Grid {
public:
  LocalCostmap();

  void setParameters(nav_msgs::msg::OccupancyGrid::SharedPtr grid,
                     const utils::EDT *edt, double robot_x, double robot_y,
                     double window_size, double inflation_radius,
                     double inscribed_radius, double scaling_factor);

  size_t getWindowWidth() const;
  size_t getWindowHeight() const;
  double getWindowOriginX() const;
  double getWindowOriginY() const;

  double getCostAt(size_t idx) const;
  double getCostAt(size_t local_x, size_t local_y) const;
  double getCostAtWorld(double world_x, double world_y) const;

  double getDistanceAt(int idx) const;
  double getDistanceAt(size_t local_x, size_t local_y) const;
  double getDistanceAtWorld(double world_x, double world_y) const;

public:
  static constexpr int COST_PRECISION = 100;
  static constexpr double LETHAL_COST = 254.0;
  static constexpr double INSCRIBED_COST = 253.0;

private:
  void computeCostmap();
  double computeCost(double dist);
  void computeDistToCostMap();

  size_t localIndex(size_t local_x, size_t local_y) const;

private:
  const utils::EDT *edt_;

  double inflation_radius_, inscribed_radius_;
  double scaling_factor_;

  size_t window_width_, window_height_;
  double window_origin_x_, window_origin_y_;
  size_t global_offset_x_, global_offset_y_;

  static constexpr double epsilon_ = 1e-6;

  std::vector<double> dist_to_cost_;

  std::vector<double> costmap_;
  std::vector<int8_t> window_data_;
};

} // namespace grid
