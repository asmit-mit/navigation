#include "utils/EDT.h"

namespace utils {

EDT::EDT() {}

void EDT::computeDT(const std::vector<int8_t> &grid, std::vector<double> &edt,
                    int height, int width) {
  int size = grid.size();

  for (int i = 0; i < size; i++)
    edt[i] = (grid[i] == 100) ? 0 : INF;

  for (int y = 0; y < height; y++)
    computeDT1D(edt, y * width, width, 1);

  for (int x = 0; x < width; x++)
    computeDT1D(edt, x, height, width);
}

void EDT::computeDT1D(std::vector<double> &edt, int start, int size,
                      int stride) {
  std::vector<int> v(size);
  std::vector<double> z(size + 1);

  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;

  auto f = [&](int q) -> double { return edt[start + q * stride]; };

  for (int q = 1; q < size; q++) {
    double s;

    while (true) {
      int vk = v[k];

      s = ((f(q) + q * q) - (f(vk) + vk * vk)) / (2.0 * (q - vk));

      if (s > z[k])
        break;

      k--;
      if (k < 0) {
        k = 0;
        break;
      }
    }

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }

  k = 0;
  for (int q = 0; q < size; q++) {
    while (z[k + 1] < q) {
      k++;
    }

    int vk = v[k];
    double dx = q - vk;

    edt[start + q * stride] = dx * dx + f(vk);
  }
}

} // namespace utils
