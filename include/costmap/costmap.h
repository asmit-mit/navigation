#pragma once

#include "costmap/grid_map.h"

#include <limits>
#include <vector>

namespace costmap {

class Costmap : public GridMap {
public:
  Costmap();

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) override;
  void setParameters(double radius, double scaling_factor);

  void computeCostmap();

  double getCostAt(int idx) const;
  double getCostAt(int x, int y) const;

private:
  void computeDT1D(int start, int size, int stride);
  void computeDT();
  double computeCost(double dist);
  void computeDistToCostMap();

private:
  double inflation_radius_, inscribed_radius_;
  double scaling_factor_;

  static constexpr double INF = std::numeric_limits<double>::infinity();
  static constexpr double epsilon_ = 1e-6;

  static constexpr int COST_PRECISION = 100;
  static constexpr double LETHAL_COST = 254.0;
  static constexpr double INSCRIBED_COST = 253.0;

  std::vector<double> dist_to_cost_;
  std::vector<double> costmap_;
};

} // namespace costmap
