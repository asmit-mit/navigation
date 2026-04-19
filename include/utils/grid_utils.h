#pragma once

#include <utility>

namespace utils {

std::pair<int, int> worldToMapDiscrete(double x, double y, double origin_x,
                                       double origin_y, double resolution);
std::pair<double, double> worldToMapContinous(double x, double y,
                                              double origin_x, double origin_y,
                                              double resolution);
std::pair<double, double> mapToWorld(double x, double y, double origin_x,
                                     double origin_y, double resolution);

} // namespace utils
