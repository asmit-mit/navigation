#include "utils/EDT.h"

#include <cmath>

namespace utils {

EDT::EDT() {}

void EDT::computeDT(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
  const int size = grid->data.size();
  const int height = grid->info.height;
  const int width = grid->info.width;

  distance_transform_.resize(size);

  int obstacle_count = 0;

  for (int i = 0; i < size; i++) {
    if (grid->data[i] == 100) {
      distance_transform_[i] = 0;
      obstacle_count++;
    } else
      distance_transform_[i] = INF;
  }

  if (obstacle_count == 0)
    return;

  for (int y = 0; y < height; y++)
    computeDT1D(y * width, width, 1);

  for (int x = 0; x < width; x++)
    computeDT1D(x, height, width);

  for (int i = 0; i < size; i++)
    distance_transform_[i] = std::sqrt(distance_transform_[i]);
}

double EDT::getDistanceAt(int idx) const { return distance_transform_[idx]; }

void EDT::computeDT1D(int start, int size, int stride) {
  std::vector<int> v(size);
  std::vector<double> z(size + 1);

  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;

  auto f = [&](int q) -> double {
    return distance_transform_[start + q * stride];
  };

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

    distance_transform_[start + q * stride] = dx * dx + f(vk);
  }
}

} // namespace utils
