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
  double distanceToNearestEdge(int x, int y);
  double getMaxDist() const;
  void ComputeFT();

private:
  int getIndex(int x, int y);
  bool isBoundary(const std::vector<int8_t> &image_, int x, int y);
  bool isValid(int x, int y);

  long long dist2(const Pixel &a, const Pixel &b);

  void ComputeF0(std::vector<int8_t> &image_,
                 std::vector<Pixel> &feature_vector_, bool compute_dsu = false);
  void ComputeF1(std::vector<Pixel> &feature_vector_, int x);
  void ComputeF2(std::vector<Pixel> &feature_vector_, int y);

  bool RemoveF1(const Pixel &u, const Pixel &v, const Pixel &w, int x);
  bool RemoveF2(const Pixel &u, const Pixel &v, const Pixel &w, int y);

private:
  DSU dsu;
  int height_, width_;
  long long max_dist2_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  std::vector<Pixel> obstacle_feature_vector_;
  std::vector<Pixel> edge_feature_vector_;

  const Pixel UNDEFINED = Pixel(-1, -1);
};

} // namespace voronoi
