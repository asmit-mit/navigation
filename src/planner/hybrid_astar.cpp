#include "planner/hybrid_astar.h"
#include "planner/motion_model.h"
#include "utils/math_utils.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_set>

using std::cout;

namespace planner {

HybridAStar::Node::Node(const Pose3d &p, HybridAStar *planner)
    : pose(p), parent(nullptr), planner(planner) {
  state = planner->poseToState(p);
  g_cost = 0;
  h_cost = 0;
}

HybridAStar::Node::Node(const Pose3d &p, HybridAStar *planner, Node *parent)
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

HybridAStar::HybridAStar() {
  optimizer_.setStepSize(0.0001);
  optimizer_.setIterations(500);

  max_angular_velocity_ = 0.5;
  max_linear_velocity_ = 2;

  angular_resolution_ = 5;
  distance_resolution_ = 0.5;

  angular_tolerance_ = 0.1;
  distance_tolerance_ = 0.5;

  motion_model_.setMotionModel(MotionModelType::DUBINS);
  motion_model_.setDistanceResolution(distance_resolution_);
  motion_model_.setMinTurningRadius(1);
}

void HybridAStar::setMotionModel(MotionModelType type) {
  motion_model_.setMotionModel(type);
}

void HybridAStar::setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
  grid_.setGrid(grid);
  height_ = grid_.getHeight();
  width_ = grid_.getWidth();
  map_resolution_ = grid_.getResolution();
}

void HybridAStar::setTolerance(double angle, double distance) {
  angular_tolerance_ = angle;
  distance_tolerance_ = distance;
}

void HybridAStar::setResolutions(double distance, double angle) {
  distance_resolution_ = distance;
  angular_resolution_ = angle;

  motion_model_.setDistanceResolution(distance_resolution_);
}

void HybridAStar::setVelocities(double linear, double angular) {
  max_linear_velocity_ = linear;
  max_angular_velocity_ = angular;

  motion_model_.setMinTurningRadius(linear / angular);

  controls_ = {{max_linear_velocity_, 0.0},
               {max_linear_velocity_, max_angular_velocity_},
               {max_linear_velocity_, -max_angular_velocity_},
               {-max_linear_velocity_, 0.0},
               {-max_linear_velocity_, max_angular_velocity_},
               {-max_linear_velocity_, -max_angular_velocity_}};
}

void HybridAStar::setGoal(double x, double y, double theta) {
  end_ = Pose3d(x, y, theta);
  preprocess();
}

void HybridAStar::setStart(double x, double y, double theta) {
  start_ = Pose3d(x, y, theta);
}

void HybridAStar::setIterations(int iterations) {
  optimizer_.setIterations(iterations);
}

std::vector<Pose2d> HybridAStar::getPlan() {
  simulate();
  freeNodes();
  return plan_;
}

State3d HybridAStar::poseToState(const Pose3d &p) {
  auto [x, y] = grid_.worldToMapDiscrete(p.x, p.y);

  double theta_deg = p.theta * 180.0 / M_PI;

  theta_deg = std::fmod(theta_deg, 360.0);
  if (theta_deg < 0.0)
    theta_deg += 360.0;

  int theta_bin = static_cast<int>(std::floor(theta_deg / angular_resolution_));

  return State3d(x, y, theta_bin);
}

void HybridAStar::preprocess() {
  holonomic_with_obstacle_cost_.assign(height_ * width_,
                                       std::numeric_limits<double>::infinity());
  auto [start_x, start_y] = grid_.worldToMapDiscrete(end_.x, end_.y);
  if (!grid_.isValid(start_x, start_y))
    return;

  int start_idx = grid_.getIndex(start_x, start_y);

  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>, std::greater<>>
      pq;

  pq.emplace(0, start_idx);
  holonomic_with_obstacle_cost_[start_idx] = 0;

  while (!pq.empty()) {
    auto [cost, idx] = pq.top();
    pq.pop();

    if (cost > holonomic_with_obstacle_cost_[idx])
      continue;

    int x = idx % width_;
    int y = idx / width_;

    for (int i = 0; i < 8; i++) {
      int new_x = x + dx_[i];
      int new_y = y + dy_[i];

      if (!grid_.isValid(new_x, new_y))
        continue;

      int new_idx = grid_.getIndex(new_x, new_y);

      double move_cost = (dx_[i] == 0 || dy_[i] == 0) ? map_resolution_
                                                      : map_resolution_ * 1.41;
      double new_cost = cost + move_cost;

      if (new_cost < holonomic_with_obstacle_cost_[new_idx]) {
        holonomic_with_obstacle_cost_[new_idx] = new_cost;
        pq.emplace(new_cost, new_idx);
      }
    }
  }
}

void HybridAStar::simulate() {
  plan_.clear();

  Node *start = new Node(start_, this);
  nodes_.push_back(start);
  start->h_cost = heuristic(start);

  std::priority_queue<Node *, std::vector<Node *>, CompareNode> open;
  std::unordered_set<Node *, NodeHash, NodeEqual> closed;
  open.push(start);

  int count = 0;

  while (!open.empty()) {
    Node *curr = open.top();
    open.pop();

    if (goalReached(curr)) {
      while (curr) {
        plan_.push_back(curr->pose);
        curr = curr->parent;
      }
      std::reverse(plan_.begin(), plan_.end());
      return;
    }

    auto it_curr = closed.find(curr);
    if (it_curr != closed.end())
      continue;
    closed.insert(curr);

    std::vector<Pose2d> analytical_expansion = analyticalExpansion(curr);
    if (!analytical_expansion.empty()) {
      while (curr) {
        plan_.push_back(curr->pose);
        curr = curr->parent;
      }
      std::reverse(plan_.begin(), plan_.end());
      plan_.insert(plan_.end(), analytical_expansion.begin(),
                   analytical_expansion.end());
      return;
    }

    std::vector<std::pair<Pose3d, double>> neighbors = expand(curr);

    for (auto [nbr, cost] : neighbors) {
      State3d next_state = poseToState(nbr);
      if (!grid_.isValid(next_state.x, next_state.y))
        continue;

      Node *next = new Node(nbr, this);
      next->g_cost = curr->g_cost + cost;
      next->h_cost = heuristic(next);
      next->parent = curr;

      open.push(next);
      nodes_.push_back(next);
    }

    count++;
    if (count > 50000)
      break;
  }

  return;
}

std::vector<Pose2d> HybridAStar::analyticalExpansion(const Node *node) {
  motion_model_.simulate(node->pose, end_);
  std::vector<Pose2d> mm_path = motion_model_.getOptimalPath();

  if (mm_path.empty())
    return {};

  for (size_t i = 1; i < mm_path.size(); ++i) {
    const Pose2d &p0 = mm_path[i - 1];
    const Pose2d &p1 = mm_path[i];

    Pose2d d = p1 - p0;

    if (i == 1) {
      State2d s0 = grid_.pose2dToState2d(p0);
      if (!grid_.isValid(s0.x, s0.y))
        return {};
    }

    for (int j = 1; j <= num_samples_; ++j) {
      double t = static_cast<double>(j) / (num_samples_ + 1);
      Pose2d interp = p0 + d * t;
      State2d s_interp = grid_.pose2dToState2d(interp);

      if (!grid_.isValid(s_interp.x, s_interp.y))
        return {};
    }

    State2d s1 = grid_.pose2dToState2d(p1);
    if (!grid_.isValid(s1.x, s1.y))
      return {};
  }

  return mm_path;
}

double HybridAStar::heuristic(const Node *node) {
  motion_model_.simulate(node->pose, end_);

  double h_mm = motion_model_.getOptimalDistance();
  double h_2d = utils::distance(node->pose, end_);

  double h1 = std::max(h_mm, h_2d);
  double h2 = holonomic_with_obstacle_cost_[grid_.getIndex(node->state.x,
                                                           node->state.y)];
  return std::max(h1, h2);
}

bool HybridAStar::goalReached(const Node *node) {
  double dtheta = std::atan2(std::sin(node->pose.theta - end_.theta),
                             std::cos(node->pose.theta - end_.theta));

  return (utils::distance(node->pose, end_) < distance_tolerance_ &&
          std::abs(dtheta) < angular_tolerance_);
}

std::vector<std::pair<Pose3d, double>> HybridAStar::expand(const Node *node) {
  std::vector<std::pair<Pose3d, double>> neighbors;

  const double step = std::min(map_resolution_, distance_resolution_);
  const double ds = step / num_samples_;

  for (auto [u, omega] : controls_) {
    Pose3d temp = node->pose;
    bool collision = false;

    for (int i = 0; i < num_samples_; ++i) {
      temp.x += ds * std::cos(temp.theta) * (u > 0 ? 1.0 : -1.0);
      temp.y += ds * std::sin(temp.theta) * (u > 0 ? 1.0 : -1.0);

      if (std::abs(omega) > 1e-6) {
        double dtheta = omega * (ds / std::abs(u));
        temp.theta += dtheta;
        temp.theta = std::atan2(std::sin(temp.theta), std::cos(temp.theta));
      }

      auto [gx, gy] = grid_.worldToMapDiscrete(temp.x, temp.y);

      if (!grid_.isValid(gx, gy)) {
        collision = true;
        break;
      }
    }

    if (!collision) {
      double cost = step;

      if (std::abs(omega) > 1e-6)
        cost *= penalty_steering_;

      if (u < 0.0)
        cost *= penalty_reverse_;

      neighbors.emplace_back(temp, cost);
    }
  }

  return neighbors;
}

void HybridAStar::freeNodes() {
  for (int i = 0; i < (int)nodes_.size(); i++)
    delete nodes_[i];
  nodes_.clear();
}

}; // namespace planner
