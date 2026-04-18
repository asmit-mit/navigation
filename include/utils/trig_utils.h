#pragma once

#include <cmath>
#include <vector>

namespace utils {

class TrigTable {
public:
  TrigTable(size_t precision = 4096);

  double sin(double theta) const;
  double cos(double theta) const;

private:
  std::vector<double> sin_table;
  std::vector<double> cos_table;

  size_t size;
  double step;
  double inv_step;

  static constexpr double TWO_PI = 2.0 * M_PI;

  inline double lerp(double a, double b, double t) const;
};

} // namespace utils
