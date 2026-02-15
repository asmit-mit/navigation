#include "planner/hybrid_astar.h"

#include <cmath>
#include <limits>
#include <queue>

namespace planner {

HybridAStar::HybridAStar(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    : grid_(grid) {
  height_ = grid->info.height;
  width_ = grid->info.width;
  resolution_ = grid->info.resolution;

  reed_shepps_.setMinTurningRadius(1);
  voronoi_.setGrid(grid_);
  voronoi_.ComputeFT();
}

void HybridAStar::setGoal(double x, double y, double theta) {
  end_ = Pose(x, y, theta);
}

void HybridAStar::setStart(double x, double y, double theta) {
  start_ = Pose(x, y, theta);
}

std::vector<Pose> HybridAStar::getPlan() {
  return {};
}

bool HybridAStar::isValid(int x, int y) {
  return x >= 0 && y >= 0 && x < width_ && y < height_ &&
         grid_->data[getIndex(x, y)] != 100;
}

int HybridAStar::getIndex(int x, int y) { return y * width_ + x; }

std::pair<int, int> HybridAStar::worldToMap(double x, double y) {
  int gx = floor((x - grid_->info.origin.position.x) / resolution_);
  int gy = floor((y - grid_->info.origin.position.y) / resolution_);
  return {gx, gy};
}

void HybridAStar::preprocess() {
  holonomic_with_obstacle_cost.assign(height_ * width_,
                                      std::numeric_limits<double>::max());
  auto [start_x, start_y] = worldToMap(end_.x, end_.y);
  if (!isValid(start_x, start_y))
    return;

  int start_idx = getIndex(start_x, start_y);

  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>, std::greater<>>
      pq;

  pq.emplace(0, start_idx);
  holonomic_with_obstacle_cost[start_idx] = 0;

  const int dx[8] = {-1, 1, -1, 0, 1, -1, 0, 1};
  const int dy[8] = {0, 0, 1, 1, 1, -1, -1, -1};

  while (!pq.empty()) {
    auto [cost, idx] = pq.top();
    pq.pop();

    if (cost > holonomic_with_obstacle_cost[idx])
      continue;

    int x = idx % width_;
    int y = idx / width_;

    for (int i = 0; i < 8; i++) {
      int new_x = x + dx[i];
      int new_y = y + dy[i];

      if (!isValid(new_x, new_y))
        continue;

      int new_idx = getIndex(new_x, new_y);

      double move_cost =
          (dx[i] == 0 || dy[i] == 0) ? resolution_ : resolution_ * 1.41;
      double new_cost = cost + move_cost;

      if (new_cost < holonomic_with_obstacle_cost[new_idx]) {
        holonomic_with_obstacle_cost[new_idx] = new_cost;
        pq.emplace(new_cost, new_idx);
      }
    }
  }
}

}; // namespace planner
