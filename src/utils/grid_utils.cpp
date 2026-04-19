#include "utils/grid_utils.h"

#include <cmath>

namespace utils {

std::pair<int, int> worldToMapDiscrete(double x, double y, double origin_x,
                                       double origin_y, double resolution) {
  int gx = (int)((x - origin_x) / resolution);
  int gy = (int)((y - origin_y) / resolution);
  return {gx, gy};
}

std::pair<double, double> worldToMapContinous(double x, double y,
                                              double origin_x, double origin_y,
                                              double resolution) {
  double gx = (x - origin_x) / resolution;
  double gy = (y - origin_y) / resolution;
  return {gx, gy};
}

std::pair<double, double> mapToWorld(double x, double y, double origin_x,
                                     double origin_y, double resolution) {
  double wx = x * resolution + origin_x;
  double wy = y * resolution + origin_y;
  return {wx, wy};
}

} // namespace utils
