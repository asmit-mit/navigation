#pragma once

#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>

#include "voronoi/DSU.h"
#include "voronoi/Pixel.h"

namespace voronoi {

class VoronoiImage {
public:
  VoronoiImage() {}

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr map);
  double distanceToNearestObstacle(int x, int y);
  void ComputeFT();
  bool isEdge(int x, int y);

private:
  int getIndex(int x, int y);
  bool isBoundary(int x, int y);
  bool isValid(int x, int y);

  long long dist2(const Pixel &a, const Pixel &b);

  void ComputeF0();
  void ComputeF1(int x);
  void ComputeF2(int y);

  bool RemoveF1(const Pixel &u, const Pixel &v, const Pixel &w, int x);
  bool RemoveF2(const Pixel &u, const Pixel &v, const Pixel &w, int y);

private:
  DSU dsu;
  int height_, width_;

  nav_msgs::msg::OccupancyGrid::SharedPtr image_;
  std::vector<Pixel> feature_vector_;

  const Pixel UNDEFINED = Pixel(-1, -1);
};

} // namespace voronoi
