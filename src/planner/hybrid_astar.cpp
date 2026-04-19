#include "planner/hybrid_astar.h"
#include "utils/grid_utils.h"
#include "utils/math_utils.h"

#include <cassert>
#include <limits>
#include <queue>

namespace planner {

HybridAStar::Node::Node() : parent(nullptr) {
  g_cost = std::numeric_limits<double>::infinity();
  h_cost = 0;
}

HybridAStar::Node::Node(const geometry::Pose3d &p) : pose(p), parent(nullptr) {
  g_cost = std::numeric_limits<double>::infinity();
  h_cost = 0;
}

HybridAStar::Node::Node(const geometry::Pose3d &p, Node *parent)
    : pose(p), parent(parent) {
  g_cost = std::numeric_limits<double>::infinity();
  h_cost = 0;
}

bool HybridAStar::CompareNode::operator()(Node *a, Node *b) {
  return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
}

HybridAStar::HybridAStar() {}

void HybridAStar::setParameters(
    const grid::GlobalCostmap *costmap, const Optimizer *optimizer,
    MotionModel *motion_model,
    const geometry::CollisionChecker *collision_checker,
    const utils::TrigTable *trig_table, const HybridAstarParams &params) {
  assert(costmap != nullptr);
  assert(optimizer != nullptr);
  assert(motion_model != nullptr);
  assert(trig_table != nullptr);
  assert(collision_checker != nullptr);

  costmap_ = costmap;
  optimizer_ = optimizer;
  motion_model_ = motion_model;
  collision_checker_ = collision_checker;
  trig_table_ = trig_table;

  controls_count_ =
      (motion_model_->getType() == MotionModelType::REED_SHEPPS) ? 6 : 3;

  height_ = costmap_->getHeight();
  width_ = costmap_->getWidth();
  map_resolution_ = costmap_->getResolution();

  expand_step_ = 1.41421356 * map_resolution_;

  // velocities
  max_angular_velocity_ = params.max_angular_velocity;
  max_linear_velocity_ = params.max_linear_velocity;

  controls_[0] = {max_linear_velocity_, 0.0};
  controls_[1] = {max_linear_velocity_, max_angular_velocity_};
  controls_[2] = {max_linear_velocity_, -max_angular_velocity_};
  controls_[3] = {-max_linear_velocity_, 0.0};
  controls_[4] = {-max_linear_velocity_, max_angular_velocity_};
  controls_[5] = {-max_linear_velocity_, -max_angular_velocity_};

  // resolutions
  angular_resolution_ = params.angular_resolution;

  // tolerance
  angular_tolerance_ = params.angular_tolerance;
  distance_tolerance_ = params.distance_tolerance;

  // analyical expansions
  analytical_expansion_ratio_ = params.analytical_expansion_ratio;
  analytical_expansion_max_length_ = params.analytical_expansion_max_length;

  // weights
  steering_penalty_ = params.steering_penalty;
  change_steering_penalty_ = params.change_steering_penalty;
  reverse_penalty_ = params.reverse_penalty;
  cost_penalty_ = params.cost_penalty;
  expansion_cost_ = params.expansion_cost;
  path_length_weight_ = params.path_length_weight;

  max_explore_iterations_ = params.max_explore_iterations;
  num_theta_bins_ = static_cast<int>(360.0 / angular_resolution_);
  state_space_size_ = height_ * width_ * num_theta_bins_;

  holonomic_with_obstacle_cost_.resize(height_ * width_,
                                       std::numeric_limits<double>::infinity());
  node_pool_.resize(state_space_size_, Node());
  closed_.resize(state_space_size_, false);
}

void HybridAStar::setGoal(double x, double y, double theta) {
  geometry::Pose3d new_goal(x, y, theta);

  bool loc_changed = utils::distance(new_goal, end_) >= distance_tolerance_;

  end_ = new_goal;

  if (loc_changed)
    buildObstacleCostTable();
}

void HybridAStar::setStart(double x, double y, double theta) {
  start_ = geometry::Pose3d(x, y, theta);
}

std::vector<geometry::Pose2d> HybridAStar::getPlan() {
  simulate();
  plan_ = optimizer_->getSmoothPath(plan_);
  return plan_;
}

geometry::State3d HybridAStar::poseToState(const geometry::Pose3d &p) {
  auto [x, y] = utils::worldToMapDiscrete(p.x, p.y, costmap_->getOriginX(),
                                          costmap_->getOriginY(),
                                          costmap_->getResolution());

  double theta_deg = p.theta * theta_to_deg_;
  theta_deg -= 360.0 * std::floor(theta_deg / 360.0);
  const size_t theta_bin =
      static_cast<int>(std::floor(theta_deg / angular_resolution_));

  return geometry::State3d(x, y, theta_bin);
}

geometry::State2d HybridAStar::pose2dToState2d(const geometry::Pose2d &p) {
  return geometry::State2d(utils::worldToMapDiscrete(
      p.x, p.y, costmap_->getOriginX(), costmap_->getOriginY(),
      costmap_->getResolution()));
}

geometry::Pose2d HybridAStar::state2dToPose2d(const geometry::State2d &s) {
  return geometry::Pose2d(utils::mapToWorld(s.x, s.y, costmap_->getOriginX(),
                                            costmap_->getOriginY(),
                                            costmap_->getResolution()));
}

size_t HybridAStar::getStateIndex(const geometry::State3d &state) const {
  return state.theta_bin * height_ * width_ + state.y * width_ + state.x;
}

geometry::State2d HybridAStar::stateIndexToState2d(size_t index) const {
  size_t area = height_ * width_;

  size_t rem = index % area;

  size_t x = rem % width_;
  size_t y = rem / width_;

  return geometry::State2d(x, y);
}

void HybridAStar::buildObstacleCostTable() {
  holonomic_with_obstacle_cost_.reset();

  const auto [start_x, start_y] = utils::worldToMapDiscrete(
      end_.x, end_.y, costmap_->getOriginX(), costmap_->getOriginY(),
      costmap_->getResolution());

  if (!costmap_->isValid(start_x, start_y))
    return;

  const size_t start_idx = costmap_->getIndex(start_x, start_y);

  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>, std::greater<>>
      pq;

  pq.emplace(0, start_idx);
  holonomic_with_obstacle_cost_.set(start_idx, 0.0);

  while (!pq.empty()) {
    const auto [cost, idx] = pq.top();
    pq.pop();

    if (cost > holonomic_with_obstacle_cost_.get(idx))
      continue;

    const size_t x = idx % width_;
    const size_t y = idx / width_;

    for (int i = 0; i < 8; i++) {
      size_t new_x = x + dx_[i];
      size_t new_y = y + dy_[i];

      if (!costmap_->isValid(new_x, new_y))
        continue;

      const size_t new_idx = costmap_->getIndex(new_x, new_y);
      const double move_cost = (dx_[i] == 0 || dy_[i] == 0)
                                   ? map_resolution_
                                   : map_resolution_ * 1.41421356;

      const double costmap_cost = costmap_->getCostAt(new_x, new_y) /
                                  (grid::GlobalCostmap::INSCRIBED_COST - 1);

      const double new_cost =
          cost + move_cost * (1.0 + cost_penalty_ * costmap_cost);

      if (new_cost < holonomic_with_obstacle_cost_.get(new_idx)) {
        holonomic_with_obstacle_cost_.set(new_idx, new_cost);
        pq.emplace(new_cost, new_idx);
      }
    }
  }
}

void HybridAStar::simulate() {
  plan_.clear();
  node_pool_.reset();
  closed_.reset();

  geometry::State3d start_state = poseToState(start_);
  const int start_idx = getStateIndex(start_state);

  Node *start = &node_pool_.get(start_idx);
  start->pose = start_;
  start->state_idx = start_idx;
  start->g_cost = 0.0;
  start->h_cost = heuristic(start);

  std::priority_queue<Node *, std::vector<Node *>, CompareNode> open;

  open.push(start);
  int count = 0;
  double accumulator = 0.0;

  while (!open.empty()) {
    Node *curr = open.top();
    open.pop();

    const int curr_idx = curr->state_idx;
    if (closed_.getByValue(curr_idx))
      continue;
    closed_.set(curr_idx, true);

    if (goalReached(curr)) {
      while (curr) {
        plan_.push_back(curr->pose);
        curr = curr->parent;
      }
      std::reverse(plan_.begin(), plan_.end());
      return;
    }

    if (accumulator >= analytical_expansion_ratio_) {
      std::vector<geometry::Pose2d> analytical_expansion =
          analyticalExpansion(curr);
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
      accumulator = 0.0;
    }

    std::vector<std::pair<geometry::Pose3d, double>> neighbors = expand(curr);
    for (auto &[nbr_pose, move_penalty] : neighbors) {
      geometry::State3d next_state = poseToState(nbr_pose);
      if (!costmap_->isValid(next_state.x, next_state.y))
        continue;

      const double costmap_cost =
          costmap_->getCostAt(next_state.x, next_state.y) /
          (grid::GlobalCostmap::INSCRIBED_COST - 1);

      const double traversal_cost =
          move_penalty * (path_length_weight_ + cost_penalty_ * costmap_cost);

      const double next_g_cost = curr->g_cost + traversal_cost;

      const size_t next_state_idx = getStateIndex(next_state);

      if (node_pool_.isSet(next_state_idx) &&
          next_g_cost >= node_pool_.get(next_state_idx).g_cost)
        continue;

      node_pool_.get(next_state_idx).g_cost = next_g_cost;

      if (closed_.getByValue(next_state_idx))
        continue;

      Node *next = &node_pool_.get(next_state_idx);

      next->pose = nbr_pose;
      next->state_idx = next_state_idx;
      next->parent = curr;

      next->g_cost = next_g_cost;
      next->h_cost = heuristic(next);

      open.push(next);
    }

    accumulator += 1.0;
    if (++count > max_explore_iterations_)
      break;
  }
}

std::vector<geometry::Pose2d>
HybridAStar::analyticalExpansion(const Node *node) {
  motion_model_->simulate(node->pose, end_);

  const double mm_dist = motion_model_->getOptimalDistance();
  if (mm_dist >= analytical_expansion_max_length_)
    return {};

  std::vector<geometry::Pose2d> mm_path = motion_model_->getOptimalPath();
  if (mm_path.empty())
    return {};

  for (size_t i = 0; i < mm_path.size(); ++i) {
    const geometry::Pose2d &p = mm_path[i];

    const geometry::State2d s = pose2dToState2d(p);
    if (collision_checker_->inCollisionGlobal(p) ||
        costmap_->getCostAt(s.x, s.y) >= expansion_cost_)
      return {};
  }

  return mm_path;
}

double HybridAStar::heuristic(const Node *node) {
  motion_model_->simulate(node->pose, end_);
  const double h_mm = motion_model_->getOptimalDistance();
  const double h_2d = utils::distance(node->pose, end_);

  const geometry::State2d state = stateIndexToState2d(node->state_idx);

  const double h1 = std::max(h_mm, h_2d);
  const double h2 =
      holonomic_with_obstacle_cost_.get(costmap_->getIndex(state.x, state.y));
  return std::max(h1, h2);
}

bool HybridAStar::goalReached(const Node *node) {
  const double dtheta = utils::M(node->pose.theta - end_.theta);
  return (utils::distance(node->pose, end_) < distance_tolerance_ &&
          std::abs(dtheta) < angular_tolerance_);
}

std::vector<std::pair<geometry::Pose3d, double>>
HybridAStar::expand(const Node *node) {
  std::vector<std::pair<geometry::Pose3d, double>> neighbors;
  neighbors.reserve(controls_count_);

  double parent_omega = 0.0;
  if (node->parent)
    parent_omega = utils::M(node->pose.theta - node->parent->pose.theta);

  for (int k = 0; k < controls_count_; k++) {
    auto [u, omega] = controls_[k];
    geometry::Pose3d temp = node->pose;

    const double dir = (u > 0) ? 1.0 : -1.0;
    temp.x += expand_step_ * trig_table_->cos(temp.theta) * dir;
    temp.y += expand_step_ * trig_table_->sin(temp.theta) * dir;

    if (std::abs(omega) > epsilon_) {
      double dtheta = omega * (expand_step_ / std::abs(u));
      temp.theta += dtheta;
      temp.theta = utils::M(temp.theta);
    }

    if (collision_checker_->inCollisionGlobal(temp))
      continue;

    double cost = expand_step_;

    bool is_steering = std::abs(omega) > epsilon_;
    bool parent_steering = std::abs(parent_omega) > epsilon_;
    bool steering_changed = node->parent && is_steering && parent_steering &&
                            (omega * parent_omega < 0.0);

    if (is_steering) {
      cost *= steering_changed ? (steering_penalty_ * change_steering_penalty_)
                               : steering_penalty_;
    }

    if (u < 0.0)
      cost *= reverse_penalty_;

    neighbors.emplace_back(temp, cost);
  }

  return neighbors;
}

}; // namespace planner
