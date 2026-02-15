#pragma once

namespace voronoi {

struct Pixel {
  int x, y;

  Pixel() {}
  Pixel(int x, int y) : x(x), y(y) {}

  bool operator==(const Pixel &other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const Pixel &other) const { return !(*this == other); }
};

} // namespace voronoi
