#pragma once

namespace grid {

struct CostmapParams {
  double inflation_radius = 0.55;
  double inscribed_radius = 0.1;
  double scaling_factor = 3.0;
  double window_size = 3.0;
};

} // namespace grid
