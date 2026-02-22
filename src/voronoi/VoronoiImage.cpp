#include "voronoi/VoronoiImage.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>

namespace voronoi {

void VoronoiImage::setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  max_dist2_ = 0;
  width_ = map->info.width;
  height_ = map->info.height;
  grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  dsu.init(width_ * height_);
  obstacle_feature_vector_.resize(width_ * height_);
  edge_feature_vector_.resize(width_ * height_);
}

void VoronoiImage::ComputeFT() {
  ComputeF0(grid_->data, obstacle_feature_vector_, true);
  for (int x = 0; x < width_; x++)
    ComputeF1(obstacle_feature_vector_, x);
  for (int y = 0; y < height_; y++)
    ComputeF2(obstacle_feature_vector_, y);

  std::fill(grid_->data.begin(), grid_->data.end(), 0);

  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      int idx = getIndex(x, y);
      if (grid_->data[idx] == 100)
        continue;

      max_dist2_ = std::max(max_dist2_,
                            dist2(Pixel(x, y), obstacle_feature_vector_[idx]));

      int root = dsu.find(getIndex(obstacle_feature_vector_[idx].x,
                                   obstacle_feature_vector_[idx].y));

      for (int i = 0; i < 4; i++) {
        int new_x = x + dx[i];
        int new_y = y + dy[i];
        if (!isValid(new_x, new_y))
          continue;

        int new_idx = getIndex(new_x, new_y);

        if (grid_->data[new_idx] == 100)
          continue;

        int new_root = dsu.find(getIndex(obstacle_feature_vector_[new_idx].x,
                                         obstacle_feature_vector_[new_idx].y));

        if (root != new_root && idx < new_idx) {
          grid_->data[idx] = 100;
        }
      }
    }
  }

  ComputeF0(grid_->data, edge_feature_vector_);
  for (int x = 0; x < width_; x++)
    ComputeF1(edge_feature_vector_, x);
  for (int y = 0; y < height_; y++)
    ComputeF2(edge_feature_vector_, y);
}

double VoronoiImage::distanceToNearestObstacle(int x, int y) {
  if (!isValid(x, y))
    return std::numeric_limits<double>::max();

  Pixel nearest = obstacle_feature_vector_[getIndex(x, y)];
  return std::sqrt(dist2(Pixel(x, y), nearest));
}

double VoronoiImage::distanceToNearestEdge(int x, int y) {
  if (!isValid(x, y))
    return std::numeric_limits<double>::max();

  Pixel nearest = edge_feature_vector_[getIndex(x, y)];
  return std::sqrt(dist2(Pixel(x, y), nearest));
}

double VoronoiImage::getMaxDist() const { return std::sqrt(max_dist2_); }

int VoronoiImage::getIndex(int x, int y) { return y * width_ + x; }

bool VoronoiImage::isBoundary(const std::vector<int8_t> &image_, int x, int y) {
  if (image_[getIndex(x, y)] != 100)
    return false;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {

      if (dx == 0 && dy == 0)
        continue;

      int nx = x + dx;
      int ny = y + dy;

      if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_)
        return true;

      if (image_[getIndex(nx, ny)] != 100)
        return true;
    }
  }

  return false;
}

bool VoronoiImage::isValid(int x, int y) {
  return x >= 0 && y >= 0 && x < width_ && y < height_;
}

long long VoronoiImage::dist2(const Pixel &a, const Pixel &b) const {
  long long dx = a.x - b.x;
  long long dy = a.y - b.y;
  return dx * dx + dy * dy;
}


void VoronoiImage::ComputeF0(std::vector<int8_t> &image_,
                             std::vector<Pixel> &feature_vector_,
                             bool compute_dsu) {
  const int dx[4] = {1, -1, 0, 1};
  const int dy[4] = {0, -1, -1, -1};

  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      int idx = getIndex(x, y);
      if (isBoundary(image_, x, y))
        feature_vector_[idx] = Pixel(x, y);
      else
        feature_vector_[idx] = UNDEFINED;

      if (!compute_dsu)
        continue;

      if (image_[idx] != 100)
        continue;

      for (int i = 0; i < 4; i++) {
        int new_x = x + dx[i];
        int new_y = y + dy[i];
        if (!isValid(new_x, new_y))
          continue;

        int new_idx = getIndex(new_x, new_y);

        if (image_[new_idx] != 100)
          continue;

        dsu.unite(idx, new_idx);
      }
    }
  }
}

void VoronoiImage::ComputeF1(std::vector<Pixel> &feature_vector_, int x) {
  std::vector<Pixel> g;
  for (int y = 0; y < height_; y++) {
    Pixel f = feature_vector_[getIndex(x, y)];
    if (f != UNDEFINED) {
      while (g.size() >= 2 &&
             RemoveF1(g[g.size() - 2], g[g.size() - 1], f, x)) {
        g.pop_back();
      }

      g.push_back(f);
    }
  }

  if (g.empty())
    return;

  int l = 0;
  for (int y = 0; y < height_; y++) {
    Pixel p = Pixel(x, y);
    while (l + 1 < (int)g.size() && dist2(p, g[l]) > dist2(p, g[l + 1]))
      l++;
    feature_vector_[getIndex(x, y)] = g[l];
  }
}

void VoronoiImage::ComputeF2(std::vector<Pixel> &feature_vector_, int y) {
  std::vector<Pixel> g;
  for (int x = 0; x < width_; x++) {
    Pixel f = feature_vector_[getIndex(x, y)];
    if (f != UNDEFINED) {
      while (g.size() >= 2 &&
             RemoveF2(g[g.size() - 2], g[g.size() - 1], f, y)) {
        g.pop_back();
      }

      g.push_back(f);
    }
  }

  if (g.empty())
    return;

  int l = 0;
  for (int x = 0; x < width_; x++) {
    Pixel p = Pixel(x, y);
    while (l + 1 < (int)g.size() && dist2(p, g[l]) > dist2(p, g[l + 1]))
      l++;
    feature_vector_[getIndex(x, y)] = g[l];
  }
}

bool VoronoiImage::RemoveF1(const Pixel &u, const Pixel &v, const Pixel &w,
                            int x) {
  long long ud = u.y;
  long long vd = v.y;
  long long wd = w.y;

  long long du = 1LL * (u.x - x) * (u.x - x);
  long long dv = 1LL * (v.x - x) * (v.x - x);
  long long dw = 1LL * (w.x - x) * (w.x - x);

  long long a = vd - ud;
  long long b = wd - vd;
  long long c = wd - ud;

  return (c * dv - b * du - a * dw - a * b * c) > 0;
}

bool VoronoiImage::RemoveF2(const Pixel &u, const Pixel &v, const Pixel &w,
                            int y) {
  long long ud = u.x;
  long long vd = v.x;
  long long wd = w.x;

  long long du = 1LL * (u.y - y) * (u.y - y);
  long long dv = 1LL * (v.y - y) * (v.y - y);
  long long dw = 1LL * (w.y - y) * (w.y - y);

  long long a = vd - ud;
  long long b = wd - vd;
  long long c = wd - ud;

  return (c * dv - b * du - a * dw - a * b * c) > 0;
}
}
; // namespace voronoi
