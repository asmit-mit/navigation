#include "planner/hybrid_astar.h"

#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

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

std::size_t HybridAStar::NodeHash::operator()(Node *node) const {
  return StateHash()(node->state);
}

bool HybridAStar::NodeEqual::operator()(Node *a, Node *b) const {
  return a->state == b->state;
}

HybridAStar::HybridAStar(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    : grid_(grid) {
  height_ = grid->info.height;
  width_ = grid->info.width;
  map_resolution_ = grid->info.resolution;

  // voronoi_.setGrid(grid_);
  // voronoi_.ComputeFT();
}

void HybridAStar::setTolerance(double angle, double distance) {
  angular_tolerance_ = angle;
  distance_tolerance_ = distance;
}

void HybridAStar::setResolutions(double distance, double angle) {
  distance_resolution_ = distance;
  angular_resolution_ = angle;

  reed_shepps_.setDistanceResolution(distance_resolution_);
}

void HybridAStar::setVelocities(double linear, double angular) {
  max_linear_velocity_ = linear;
  max_angular_velocity_ = angular;

  reed_shepps_.setMinTurningRadius(linear / angular);

  controls = {{max_linear_velocity_, 0.0},
              {max_linear_velocity_, max_angular_velocity_},
              {max_linear_velocity_, -max_angular_velocity_},
              {-max_linear_velocity_, 0.0},
              {-max_linear_velocity_, max_angular_velocity_},
              {-max_linear_velocity_, -max_angular_velocity_}};
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
  nodes.push_back(start);
  start->h_cost = heuristic(start);

  std::priority_queue<Node *, std::vector<Node *>, CompareNode> open;
  std::unordered_set<Node *, NodeHash, NodeEqual> closed;
  open.push(start);

  int count = 0;

  while (!open.empty()) {
    Node *curr = open.top();
    open.pop();

    if (goalReached(curr)) {
      std::vector<Pose> path;
      while (curr) {
        path.push_back(curr->pose);
        curr = curr->parent;
      }
      std::reverse(path.begin(), path.end());
      freeNodes();
      return path;
    }

    auto it_curr = closed.find(curr);
    if (it_curr != closed.end())
      continue;
    closed.insert(curr);

    std::vector<Pose> analytical_expansion = analyticalExpansion(curr);
    if (!analytical_expansion.empty()) {
      std::vector<Pose> path;
      while (curr) {
        path.push_back(curr->pose);
        curr = curr->parent;
      }
      std::reverse(path.begin(), path.end());
      path.insert(path.end(), analytical_expansion.begin(),
                  analytical_expansion.end());
      freeNodes();
      return path;
    }

    std::vector<std::pair<Pose, double>> neighbors = expand(curr);

    for (auto [nbr, cost] : neighbors) {
      Node *next = new Node(nbr, this);

      if (!isValid(next->state.grid_x, next->state.grid_y)) {
        delete next;
        continue;
      }

      next->g_cost = curr->g_cost + cost;
      next->h_cost = heuristic(next);
      next->parent = curr;

      open.push(next);
      nodes.push_back(next);
    }

    count++;
    if (count > 50000)
      break;
  }

  freeNodes();

  return {};
}

bool HybridAStar::isValid(int x, int y) {
  return x >= 0 && y >= 0 && x < width_ && y < height_ &&
         grid_->data[getIndex(x, y)] != 100;
}

int HybridAStar::getIndex(int x, int y) { return y * width_ + x; }

std::pair<int, int> HybridAStar::worldToMapDiscrete(double x, double y) {
  int gx = floor((x - grid_->info.origin.position.x) / map_resolution_);
  int gy = floor((y - grid_->info.origin.position.y) / map_resolution_);
  return {gx, gy};
}

std::pair<double, double> HybridAStar::worldToMapContinous(double x, double y) {
  double gx = (x - grid_->info.origin.position.x) / map_resolution_;
  double gy = (y - grid_->info.origin.position.y) / map_resolution_;
  return {gx, gy};
}

std::pair<double, double> HybridAStar::mapToWorld(double x, double y) {
  double wx = x * map_resolution_ + grid_->info.origin.position.x;
  double wy = y * map_resolution_ + grid_->info.origin.position.y;
  return {wx, wy};
}

State HybridAStar::poseToState(const Pose &p) {
  auto [x, y] = worldToMapDiscrete(p.x, p.y);

  double theta_deg = p.theta * 180.0 / M_PI;

  theta_deg = std::fmod(theta_deg, 360.0);
  if (theta_deg < 0.0)
    theta_deg += 360.0;

  int theta_bin = static_cast<int>(std::floor(theta_deg / angular_resolution_));

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

      double move_cost =
          (dx[i] == 0 || dy[i] == 0) ? map_resolution_ : map_resolution_ * 1.41;
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

std::vector<Pose> HybridAStar::analyticalExpansion(const Node *node) {
  reed_shepps_.simulate(node->pose, end_);
  std::vector<Pose> rs_path = reed_shepps_.getOptimalPath();

  if (rs_path.empty())
    return {};

  const double step = distance_resolution_;
  const double sample_ds = map_resolution_ * 0.2;
  const int num_samples = std::ceil(step / sample_ds);

  for (size_t i = 1; i < rs_path.size(); ++i) {
    const Pose &p0 = rs_path[i - 1];
    const Pose &p1 = rs_path[i];

    if (i == 1) {
      auto [gx0, gy0] = worldToMapDiscrete(p0.x, p0.y);
      if (!isValid(gx0, gy0))
        return {};
    }

    for (int j = 1; j <= num_samples; ++j) {
      double t = static_cast<double>(j) / (num_samples + 1);

      Pose interp;
      interp.x = p0.x + t * (p1.x - p0.x);
      interp.y = p0.y + t * (p1.y - p0.y);

      double dtheta = std::atan2(std::sin(p1.theta - p0.theta),
                                 std::cos(p1.theta - p0.theta));
      interp.theta = p0.theta + t * dtheta;
      interp.theta = std::atan2(std::sin(interp.theta), std::cos(interp.theta));

      auto [gx, gy] = worldToMapDiscrete(interp.x, interp.y);

      if (!isValid(gx, gy))
        return {};
    }

    auto [gx1, gy1] = worldToMapDiscrete(p1.x, p1.y);
    if (!isValid(gx1, gy1))
      return {};
  }

  return rs_path;
}
double HybridAStar::heuristic(const Node *node) {
  reed_shepps_.simulate(node->pose, end_);

  double h_rs = reed_shepps_.getOptimalDistance();
  double h_2d = distance(node->pose, end_);

  double h1 = std::max(h_rs, h_2d);
  double h2 = holonomic_with_obstacle_cost[getIndex(node->state.grid_x,
                                                    node->state.grid_y)];
  return std::max(h1, h2);
}

bool HybridAStar::goalReached(const Node *node) {
  double dtheta = std::atan2(std::sin(node->pose.theta - end_.theta),
                             std::cos(node->pose.theta - end_.theta));

  return (distance(node->pose, end_) < distance_tolerance_ &&
          std::abs(dtheta) < angular_tolerance_);
}

std::vector<std::pair<Pose, double>> HybridAStar::expand(const Node *node) {
  std::vector<std::pair<Pose, double>> neighbors;

  const double step = distance_resolution_;
  const double sample_ds = map_resolution_ * 0.2;
  const int num_samples = std::ceil(step / sample_ds);
  const double ds = step / num_samples;

  const double penalty_steering = 1.05;
  const double penalty_reverse = 3.0;

  for (auto [u, omega] : controls) {
    Pose temp = node->pose;
    bool collision = false;

    for (int i = 0; i < num_samples; ++i) {
      temp.x += ds * std::cos(temp.theta) * (u > 0 ? 1.0 : -1.0);
      temp.y += ds * std::sin(temp.theta) * (u > 0 ? 1.0 : -1.0);

      if (std::abs(omega) > 1e-6) {
        double dtheta = omega * (ds / std::abs(u));
        temp.theta += dtheta;
        temp.theta = std::atan2(std::sin(temp.theta), std::cos(temp.theta));
      }

      auto [gx, gy] = worldToMapDiscrete(temp.x, temp.y);

      if (!isValid(gx, gy)) {
        collision = true;
        break;
      }
    }

    if (!collision) {
      double cost = step;

      if (std::abs(omega) > 1e-6)
        cost *= penalty_steering;

      if (u < 0.0)
        cost *= penalty_reverse;

      neighbors.emplace_back(temp, cost);
    }
  }

  return neighbors;
}

void HybridAStar::freeNodes() {
  for (int i = 0; i < (int)nodes.size(); i++) {
    delete nodes[i];
  }
}

}; // namespace planner
