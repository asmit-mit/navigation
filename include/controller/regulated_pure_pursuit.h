#pragma once

#include <vector>

#include "controller/parameters.h"
#include "geometry/collison_checker.h"
#include "geometry/pose.h"
#include "grid/local_costmap.h"
#include "utils/trig_utils.h"

namespace controller {

class RegulatedPurePursuit {
public:
  RegulatedPurePursuit();

  void setParameters(const grid::LocalCostmap *costmap,
                     const geometry::CollisionChecker *collison_checker,
                     const utils::TrigTable *trig_table,
                     controller::ControllerParams &params);

  std::pair<double, double> computeCommand(const geometry::Pose3d &curr_pose,
                                           const geometry::Pose3d &goal_pose,
                                           double linear_velocity,
                                           double angular_velocity,
                                           const std::vector<geometry::Pose3d> &plan);
  geometry::Pose3d getLookaheadPoint() const;

private:
  geometry::Pose3d findLookaheadPoint(const geometry::Pose3d &curr_pose,
                                      const std::vector<geometry::Pose3d> &plan,
                                      double lookahead_dist) const;
  double computeCurvature(const geometry::Pose3d &curr_pose,
                          const geometry::Pose3d &lookahead_point) const;
  double regulateByCurvature(double v, double curvature) const;
  double regulateByCostmap(double v, const geometry::Pose3d &curr_pose) const;
  double regulateByGoalProximity(double v,
                                 const geometry::Pose3d &curr_pose,
                                 const std::vector<geometry::Pose3d> &plan) const;

  double angleToTarget(const geometry::Pose3d &curr_pose, const geometry::Pose3d &target) const;
  bool shouldRotateToGoalHeading(const geometry::Pose3d &curr_pose,
                                 const geometry::Pose3d &goal) const;
  bool checkCollision(const geometry::Pose3d &curr_pose,
                      double linear_velocity,
                      double angular_velocity,
                      double lookahead_dist) const;

  std::pair<double, double> applyDynamicWindow(double curr_linear_vel,
                                               double curr_angular_vel,
                                               double regulated_vel,
                                               double curvature);

  std::pair<double, double> rotateToHeading(double angle_to_lookahead,
                                            double angular_velocity) const;
  double computeLookaheadDistance(double linear_velocity) const;
  double computeCuspDistance(double lookahead_dist, const std::vector<geometry::Pose3d> &plan);

private:
  double min_linear_vel_;
  double max_linear_vel_, max_angular_vel_;
  double max_linear_accel_, max_angular_accel_;
  double sim_time_;

  double lookahead_distance_, lookahead_gain_;
  double max_lookahead_distance_, min_lookahead_distance_;

  double proximity_distance_, proximity_heurisitc_scale_;

  double approach_velocity_scaling_dist_;
  double min_heading_angle_error_;

  double distance_tolerance_;

  double min_turning_radius_;
  double T_k_;
  double dt_;

  static constexpr double epsilon_ = 1e-6;

  geometry::Pose3d lookahead_point_;
  const grid::LocalCostmap *costmap_;
  const geometry::CollisionChecker *collision_checker_;
  const utils::TrigTable *trig_table_;
};

} // namespace controller
