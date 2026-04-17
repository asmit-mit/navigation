#include "controller/regulated_pure_pursuit.h"
#include "utils/math_utils.h"

namespace controller {

RegulatedPurePursuit::RegulatedPurePursuit() {}

void RegulatedPurePursuit::setParameters(const grid::LocalCostmap *costmap,
                                         const utils::TrigTable *trig_table,
                                         controller::ControllerParams &params) {
  costmap_ = costmap;
  trig_table_ = trig_table;

  max_linear_velocity_ = params.max_linear_velocity;
  max_angular_velocity_ = params.max_angular_velocity;
  lookahead_distance_ = params.lookahead_distance;
  lookahead_gain_ = params.lookahead_gain;
  max_lookahead_distance_ = params.max_lookahead_distance;
  min_lookahead_distance_ = params.min_lookahead_distance;
  proximity_distance_ = params.proximity_distance;
  approach_velocity_scaling_dist_ = params.approach_velocity_scaling_dist;
  min_approach_linear_velocity_ = params.min_approach_linear_velocity;

  proximity_heurisitc_scale_ = std::min(params.proximity_heurisitc_scale, 1.0);

  distance_tolerance_ = params.distance_tolerance;

  min_turning_radius_ = max_linear_velocity_ / max_angular_velocity_;
  T_k_ = 1.0 / min_turning_radius_;
}

std::pair<double, double> RegulatedPurePursuit::computeCommand(
    const geometry::Pose3d &curr_pose, double linear_velocity,
    const std::vector<geometry::Pose2d> &plan) {
  if (plan.empty() || goalReached(curr_pose, plan.back()))
    return {0.0, 0.0};

  double lookahead_dist = computeLookaheadDistance(linear_velocity);

  geometry::Pose2d lookahead_point =
      findLookaheadPoint(curr_pose, plan, lookahead_dist);

  // add constraints here

  double k = computeCurvature(curr_pose, lookahead_point);

  double v = max_linear_velocity_;
  v = std::min(v, regulateByCurvature(v, k));
  v = std::min(v, regulateByCostmap(v, curr_pose));
  v = std::min(v, regulateByGoalProximity(v, curr_pose, plan));

  double w = k * v;
  w = std::clamp(w, -max_angular_velocity_, max_angular_velocity_);

  return {v, w};
}

geometry::Pose2d RegulatedPurePursuit::findLookaheadPoint(
    const geometry::Pose3d &curr_pose,
    const std::vector<geometry::Pose2d> &plan, double lookahead_dist) {

  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan.size(); ++i) {
    double dist = utils::distance(curr_pose, plan[i]);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  for (size_t i = closest_idx; i < plan.size(); ++i) {
    double dist = utils::distance(curr_pose, plan[i]);
    if (dist >= lookahead_dist)
      return plan[i];
  }

  return plan.back();
}

double RegulatedPurePursuit::computeCurvature(
    const geometry::Pose3d &curr_pose,
    const geometry::Pose2d &lookahead_point) {

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

double RegulatedPurePursuit::regulateByCurvature(double v, double curvature) {
  double k = std::abs(curvature);
  if (k < epsilon_ || k <= T_k_)
    return v;

  double regulated = v / (min_turning_radius_ * k);
  return std::min(regulated, v);
}

double
RegulatedPurePursuit::regulateByCostmap(double v,
                                        const geometry::Pose3d &curr_pose) {
  double d_O = costmap_->getDistanceAtWorld(curr_pose.x, curr_pose.y);
  if (d_O < 0.0 || d_O > proximity_distance_)
    return v;

  double scale = proximity_heurisitc_scale_ * (d_O / proximity_distance_);
  scale = std::clamp(scale, 0.0, 1.0);

  return std::max(v * scale, min_approach_linear_velocity_);
}

double RegulatedPurePursuit::regulateByGoalProximity(
    double v, const geometry::Pose3d &curr_pose,
    const std::vector<geometry::Pose2d> &plan) {
  double dist_to_goal = utils::distance(curr_pose, plan.back());
  if (dist_to_goal >= approach_velocity_scaling_dist_)
    return v;

  double scale = dist_to_goal / approach_velocity_scaling_dist_;
  double approach_vel = v * scale;

  approach_vel = std::max(approach_vel, min_approach_linear_velocity_);

  return std::min(v, approach_vel);
}

bool RegulatedPurePursuit::goalReached(const geometry::Pose3d &curr_pose,
                                       const geometry::Pose2d &end) const {
  return utils::distance(curr_pose, end) < distance_tolerance_;
}

double RegulatedPurePursuit::computeLookaheadDistance(double linear_velocity) {
  return std::clamp(lookahead_gain_ * linear_velocity, min_lookahead_distance_,
                    max_lookahead_distance_);
}

} // namespace controller
