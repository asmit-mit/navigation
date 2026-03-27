#pragma once

#include "planner/grid_map.h"

#include <limits>
#include <vector>

namespace costmap {

class Costmap {
public:
  Costmap() {}

  void setGrid(const planner::GridMap *grid);
  void setRadius(double radius);
  void setScalingFactor(double scaling_factor);
  void computeCostmap();

  double getDataAt(int x, int y) const;
  double getDataAt(int idx) const;

private:
  void computeDT1D(int start, int size, int stride);

private:
  const planner::GridMap *grid_;
  double radius_, scaling_factor_;

  static constexpr double INF = std::numeric_limits<double>::infinity();

  std::vector<double> costmap_;
};

} // namespace costmap
