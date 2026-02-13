#pragma once

namespace voronoi {

struct Pixel {
  int x, y;
  bool is_edge;

  Pixel() : is_edge(false) {}
  Pixel(int x, int y) : x(x), y(y), is_edge(false) {}

  bool operator==(const Pixel &other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const Pixel &other) const { return !(*this == other); }
};

} // namespace voronoi
