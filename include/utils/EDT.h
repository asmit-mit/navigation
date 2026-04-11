#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace utils {

class EDT {
public:
  EDT();

  static void computeDT(const std::vector<int8_t> &grid,
                        std::vector<double> &edt, int height, int width);

private:
  static void computeDT1D(std::vector<double> &edt, int start, int size,
                          int stride);

private:
  static constexpr double INF = std::numeric_limits<double>::infinity();
};

} // namespace utils
