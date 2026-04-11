#pragma once

#include "grid/grid.h"

#include <vector>

namespace grid {

class Costmap : public Grid {
public:
  Costmap();

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) override;
  void setParameters(double radius, double scaling_factor);

  void computeCostmap();

  double getCostAt(int idx) const;
  double getCostAt(int x, int y) const;

public:
  static constexpr int COST_PRECISION = 100;
  static constexpr double LETHAL_COST = 254.0;
  static constexpr double INSCRIBED_COST = 253.0;

private:
  double computeCost(double dist);
  void computeDistToCostMap();

private:
  double inflation_radius_, inscribed_radius_;
  double scaling_factor_;

  static constexpr double epsilon_ = 1e-6;

  std::vector<double> dist_to_cost_;
  std::vector<double> costmap_;
};

} // namespace grid
