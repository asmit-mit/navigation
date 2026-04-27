#include "controller/regulated_pure_pursuit.h"
#include "utils/math_utils.h"

#include <iostream>
#include <limits>

namespace controller {

RegulatedPurePursuit::RegulatedPurePursuit() {}

void RegulatedPurePursuit::setParameters(const grid::LocalCostmap *costmap,
                                         const geometry::CollisionChecker *collision_checker,
                                         const utils::TrigTable *trig_table,
                                         controller::ControllerParams &params) {
  costmap_ = costmap;
  trig_table_ = trig_table;
  collision_checker_ = collision_checker;

  max_linear_vel_ = params.max_linear_velocity;
  max_angular_vel_ = params.max_angular_velocity;
  max_linear_accel_ = params.max_linear_acceleration;
  max_angular_accel_ = params.max_angular_acceleration;
  sim_time_ = params.sim_time;
  lookahead_distance_ = params.lookahead_distance;
  lookahead_gain_ = params.lookahead_gain;
  max_lookahead_distance_ = params.max_lookahead_distance;
  min_lookahead_distance_ = params.min_lookahead_distance;
  proximity_distance_ = params.proximity_distance;
  approach_velocity_scaling_dist_ = params.approach_velocity_scaling_dist;
  min_linear_vel_ = params.min_approach_linear_velocity;
  min_heading_angle_error_ = params.min_heading_angle_error;

  proximity_heurisitc_scale_ = std::min(params.proximity_heuristic_scale, 1.0);

  distance_tolerance_ = params.distance_tolerance;

  min_turning_radius_ = max_linear_vel_ / max_angular_vel_;
  T_k_ = 1.0 / min_turning_radius_;

  dt_ = 1.0 / params.controller_frequency;
}

std::pair<double, double>
RegulatedPurePursuit::computeCommand(const geometry::Pose3d &curr_pose,
                                     const geometry::Pose3d &goal_pose,
                                     double linear_velocity,
                                     double angular_velocity,
                                     const std::vector<geometry::Pose2d> &plan) {
  if (shouldRotateToGoalHeading(curr_pose, goal_pose)) {
    double angle_to_goal = utils::M(goal_pose.theta - curr_pose.theta);
    return rotateToHeading(angle_to_goal, angular_velocity);
  }

  if (plan.empty())
    return {0.0, 0.0};

  size_t closest_idx = findCosestIdx(curr_pose, plan);

  double lookahead_dist = computeLookaheadDistance(linear_velocity);
  double cusp_dist = computeCuspDistance(curr_pose, closest_idx, lookahead_dist, plan);
  lookahead_dist = std::min(lookahead_dist, cusp_dist);

  lookahead_point_ = findLookaheadPoint(curr_pose, lookahead_dist, closest_idx, plan);

  double dx = lookahead_point_.x - curr_pose.x;
  double dy = lookahead_point_.y - curr_pose.y;

  double cos_yaw = trig_table_->cos(curr_pose.theta);
  double sin_yaw = trig_table_->sin(curr_pose.theta);

  double x_r = cos_yaw * dx + sin_yaw * dy;
  double sign = (x_r >= 0.0) ? 1.0 : -1.0;
  // std::cerr << "Sign:" << sign << "\n";

  double k = sign * computeCurvature(curr_pose, lookahead_point_);

  double angle_to_lookahead = angleToTarget(curr_pose, lookahead_point_);
  if (sign >= 0.0 && std::abs(angle_to_lookahead) > min_heading_angle_error_)
    return rotateToHeading(angle_to_lookahead, angular_velocity);

  double v = max_linear_vel_;
  v = std::min(v, regulateByCurvature(v, k));
  v = std::min(v, regulateByCostmap(v, curr_pose));
  v = std::min(v, regulateByGoalProximity(v, curr_pose, plan));

  double best_v, best_w;
  const double min_feasible = std::abs(linear_velocity) - max_linear_accel_ * dt_;
  const double max_feasible = std::abs(linear_velocity) + max_linear_accel_ * dt_;
  best_v = sign * std::clamp(v, std::max(0.0, min_feasible), max_feasible);
  best_w = std::clamp(k * best_v, -max_angular_vel_, max_angular_vel_);

  if (checkCollision(curr_pose, best_v, best_w, lookahead_dist)) {
    std::cerr << "Collision Detected\n";
    return {linear_velocity, angular_velocity};
  }

  return {best_v, best_w};
}

size_t RegulatedPurePursuit::findCosestIdx(const geometry::Pose3d &curr_pose,
                                           const std::vector<geometry::Pose2d> &plan) {
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan.size(); ++i) {
    double dist = utils::distance(curr_pose, plan[i]);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

geometry::Pose2d
RegulatedPurePursuit::findLookaheadPoint(const geometry::Pose3d &curr_pose,
                                         double lookahead_dist,
                                         size_t closest_idx,
                                         const std::vector<geometry::Pose2d> &plan) const {
  for (size_t i = closest_idx; i < plan.size(); ++i) {
    double dist = utils::distance(curr_pose, plan[i]);
    if (dist >= lookahead_dist)
      return plan[i];
  }

  return plan.back();
}

geometry::Pose2d RegulatedPurePursuit::getLookaheadPoint() const { return lookahead_point_; }

double RegulatedPurePursuit::computeCurvature(const geometry::Pose3d &curr_pose,
                                              const geometry::Pose2d &lookahead_point) const {
  double dx = lookahead_point.x - curr_pose.x;
  double dy = lookahead_point.y - curr_pose.y;
  double theta = curr_pose.theta;

  double x = trig_table_->cos(theta) * dx + trig_table_->sin(theta) * dy;
  double y = -trig_table_->sin(theta) * dx + trig_table_->cos(theta) * dy;

  double l2 = x * x + y * y;
  if (l2 < epsilon_)
    return 0.0;

  return 2.0 * y / l2;
}

double RegulatedPurePursuit::regulateByCurvature(double v, double curvature) const {
  double k = std::abs(curvature);
  if (k < epsilon_ || k <= T_k_)
    return v;

  double regulated = v / (min_turning_radius_ * k);
  return std::min(regulated, v);
}

double RegulatedPurePursuit::regulateByCostmap(double v, const geometry::Pose3d &curr_pose) const {
  double d_O = costmap_->getDistanceAtWorld(curr_pose.x, curr_pose.y);
  if (d_O < 0.0 || d_O > proximity_distance_)
    return v;

  double scale = proximity_heurisitc_scale_ * (d_O / proximity_distance_);
  scale = std::clamp(scale, 0.0, 1.0);

  return std::max(v * scale, min_linear_vel_);
}

double RegulatedPurePursuit::regulateByGoalProximity(
    double v, const geometry::Pose3d &curr_pose, const std::vector<geometry::Pose2d> &plan) const {
  double dist_to_goal = utils::distance(curr_pose, plan.back());
  if (dist_to_goal >= approach_velocity_scaling_dist_)
    return v;

  double scale = dist_to_goal / approach_velocity_scaling_dist_;
  double approach_vel = v * scale;

  approach_vel = std::max(approach_vel, min_linear_vel_);

  return std::min(v, approach_vel);
}

double RegulatedPurePursuit::angleToTarget(const geometry::Pose3d &curr_pose,
                                           const geometry::Pose2d &target) const {
  double dx = target.x - curr_pose.x;
  double dy = target.y - curr_pose.y;
  double bearing = std::atan2(dy, dx);
  return utils::M(bearing - curr_pose.theta);
}

std::pair<double, double> RegulatedPurePursuit::rotateToHeading(double angle_to_lookahead,
                                                                double curr_angular_vel) const {
  const double sign = angle_to_lookahead > 0.0 ? 1.0 : -1.0;
  double angular_vel = sign * max_angular_vel_;

  const double min_feasible = curr_angular_vel - max_angular_accel_ * dt_;
  const double max_feasible = curr_angular_vel + max_angular_accel_ * dt_;
  angular_vel = std::clamp(angular_vel, min_feasible, max_feasible);

  double max_vel_to_stop = std::sqrt(2.0 * max_angular_accel_ * std::abs(angle_to_lookahead));
  if (std::abs(angular_vel) > max_vel_to_stop)
    angular_vel = sign * max_vel_to_stop;

  return {0.0, angular_vel};
}

bool RegulatedPurePursuit::shouldRotateToGoalHeading(const geometry::Pose3d &curr_pose,
                                                     const geometry::Pose3d &goal) const {
  return utils::distance(curr_pose, goal) < distance_tolerance_ * 2.0;
}

bool RegulatedPurePursuit::checkCollision(const geometry::Pose3d &curr_pose,
                                          double linear_vel,
                                          double angular_vel,
                                          double lookahead_dist) const {
  if (collision_checker_->inCollisionLocal(curr_pose))
    return true;

  geometry::Pose3d robot_pose = curr_pose;

  const double dt = 1.41421356 * costmap_->getResolution();

  double t = 0.0;
  while (t < sim_time_) {
    robot_pose.x += linear_vel * dt * trig_table_->cos(robot_pose.theta);
    robot_pose.y += linear_vel * dt * trig_table_->sin(robot_pose.theta);
    robot_pose.theta += angular_vel * dt;

    if (utils::distance(curr_pose, robot_pose) > lookahead_dist)
      break;

    if (collision_checker_->inCollisionLocal(robot_pose))
      return true;

    t += dt;
  }

  return false;
}

double RegulatedPurePursuit::computeLookaheadDistance(double linear_velocity) const {
  return std::clamp(lookahead_gain_ * std::abs(linear_velocity),
                    min_lookahead_distance_,
                    max_lookahead_distance_);
}

double RegulatedPurePursuit::computeCuspDistance(const geometry::Pose3d &curr_pose,
                                                 size_t closest_idx,
                                                 double lookahead_dist,
                                                 const std::vector<geometry::Pose2d> &plan) {
  if (plan.size() < 3 || closest_idx >= plan.size() - 2)
    return std::numeric_limits<double>::infinity();

  for (size_t i = closest_idx + 1; i + 1 < plan.size(); ++i) {
    double dx1 = plan[i].x - plan[i - 1].x;
    double dy1 = plan[i].y - plan[i - 1].y;
    double dx2 = plan[i + 1].x - plan[i].x;
    double dy2 = plan[i + 1].y - plan[i].y;

    double dot = (dx1 * dx2 + dy1 * dy2);

    double dist = utils::distance(curr_pose, plan[i]);

    if (dist >= lookahead_dist)
      return lookahead_dist;

    if (dot < epsilon_)
      return dist;
  }

  return std::numeric_limits<double>::infinity();
}

} // namespace controller
