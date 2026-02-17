#include "planner/hybrid_astar.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>

namespace planner {

HybridAStar::Node::Node(const Pose &p, HybridAStar *planner)
    : pose(p), parent(nullptr), planner(planner) {
  state = planner->poseToState(p);
  g_cost = 0;
  h_cost = 0;
}

HybridAStar::Node::Node(const Pose &p, HybridAStar *planner, Node *parent)
    : pose(p), parent(parent), planner(planner) {
  state = planner->poseToState(p);
  g_cost = 0;
  h_cost = 0;
}

bool HybridAStar::CompareNode::operator()(Node *a, Node *b) {
  return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
}

std::size_t HybridAStar::NodeHash::operator()(Node *node) {
  return StateHash()(node->state);
}

bool HybridAStar::NodeEqual::operator()(Node *a, Node *b) {
  return a->state == b->state;
}

HybridAStar::HybridAStar(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    : grid_(grid) {
  height_ = grid->info.height;
  width_ = grid->info.width;
  distance_resolution_ = grid->info.resolution;

  reed_shepps_.setMinTurningRadius(1);
  voronoi_.setGrid(grid_);
  voronoi_.ComputeFT();
}

void HybridAStar::setTolerance(double angle, double distance) {
  angle_tolerance_ = angle;
  distance_tolerance_ = distance;
}

void HybridAStar::setAngularResolution(double angle) {
  angle_resolution_ = angle;
}

void HybridAStar::setGoal(double x, double y, double theta) {
  end_ = Pose(x, y, theta);
  preprocess();
}

void HybridAStar::setStart(double x, double y, double theta) {
  start_ = Pose(x, y, theta);
}

std::vector<Pose> HybridAStar::getPlan() {
  Node *start = new Node(start_, this);
  Node *end = new Node(end_, this);

  open.push(start);
  closed[start] = 0;

  while (!open.empty()) {
    Node *curr = open.top();
    open.pop();

    if (curr->g_cost >= closed[curr])
      continue;

    if (start->state == end->state) {
      // reconstruct path and return
      return {};
    }


  }

  return {};
}

bool HybridAStar::isValid(int x, int y) {
  return x >= 0 && y >= 0 && x < width_ && y < height_ &&
         grid_->data[getIndex(x, y)] != 100;
}

int HybridAStar::getIndex(int x, int y) { return y * width_ + x; }

std::pair<int, int> HybridAStar::worldToMapDiscrete(double x, double y) {
  int gx = floor((x - grid_->info.origin.position.x) / distance_resolution_);
  int gy = floor((y - grid_->info.origin.position.y) / distance_resolution_);
  return {gx, gy};
}

std::pair<double, double> HybridAStar::worldToMapContinous(double x, double y) {
  int gx = (x - grid_->info.origin.position.x) / distance_resolution_;
  int gy = (y - grid_->info.origin.position.y) / distance_resolution_;
  return {gx, gy};
}

std::pair<double, double> HybridAStar::mapToWorld(double x, double y) {
  int wx = x * distance_resolution_ + grid_->info.origin.position.x;
  int wy = y * distance_resolution_ + grid_->info.origin.position.y;
  return {wx, wy};
}

State HybridAStar::poseToState(const Pose &p) {
  auto [x, y] = worldToMapDiscrete(p.x, p.y);

  double theta_deg = p.theta * 180.0 / M_PI;

  theta_deg = std::fmod(theta_deg, 360.0);
  if (theta_deg < 0.0)
    theta_deg += 360.0;

  int theta_bin = static_cast<int>(std::floor(theta_deg / angle_resolution_));

  return State(x, y, theta_bin);
}

void HybridAStar::preprocess() {
  holonomic_with_obstacle_cost.assign(height_ * width_,
                                      std::numeric_limits<double>::infinity());
  auto [start_x, start_y] = worldToMapDiscrete(end_.x, end_.y);
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

      double move_cost = (dx[i] == 0 || dy[i] == 0)
                             ? distance_resolution_
                             : distance_resolution_ * 1.41;
      double new_cost = cost + move_cost;

      if (new_cost < holonomic_with_obstacle_cost[new_idx]) {
        holonomic_with_obstacle_cost[new_idx] = new_cost;
        pq.emplace(new_cost, new_idx);
      }
    }
  }
}

double HybridAStar::distance(const Pose &a, const Pose &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double HybridAStar::heuristic(const Node *a) {
  auto [x, y] = mapToWorld(a->pose.x, a->pose.y);
  Pose start(x, y, a->pose.theta);

  double h_rs = reed_shepps_.getOptimalPath(start, end_);
  double h_2d = distance(start, end_);

  double h1 = std::max(h_rs, h_2d);
  double h2 =
      holonomic_with_obstacle_cost[getIndex(a->state.grid_x, a->state.grid_y)];
  return std::max(h1, h2);
}

}; // namespace planner
