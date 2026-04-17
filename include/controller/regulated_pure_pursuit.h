#pragma once

#include <vector>

#include "controller/parameters.h"
#include "geometry/pose.h"
#include "grid/local_costmap.h"
#include "utils/trig_utils.h"

namespace controller {

class RegulatedPurePursuit {
public:
  RegulatedPurePursuit();

  void setParameters(const grid::LocalCostmap *costmap,
                     const utils::TrigTable *trig_table,
                     controller::ControllerParams &params);

  std::pair<double, double>
  computeCommand(const geometry::Pose3d &curr_pose, double linear_velocity,
                 const std::vector<geometry::Pose2d> &plan);

private:
  geometry::Pose2d findLookaheadPoint(const geometry::Pose3d &curr_pose,
                                      const std::vector<geometry::Pose2d> &plan,
                                      double lookahead_dist);
  double computeCurvature(const geometry::Pose3d &curr_pose,
                          const geometry::Pose2d &lookahead_point);

  double regulateByCurvature(double v, double curvature);
  double regulateByCostmap(double v, const geometry::Pose3d &curr_pose);
  double regulateByGoalProximity(double v, const geometry::Pose3d &curr_pose,
                                 const std::vector<geometry::Pose2d> &plan);

  bool goalReached(const geometry::Pose3d &curr_pose,
                   const geometry::Pose2d &end) const;
  double computeLookaheadDistance(double linear_velocity);

private:
  double max_linear_velocity_;
  double max_angular_velocity_;

  double lookahead_distance_;
  double lookahead_gain_;
  double max_lookahead_distance_;
  double min_lookahead_distance_;

  double proximity_distance_;
  double proximity_heurisitc_scale_;

  double approach_velocity_scaling_dist_;
  double min_approach_linear_velocity_;

  double distance_tolerance_;

  double min_turning_radius_;
  double T_k_;

  static constexpr double epsilon_ = 1e-6;

  const grid::LocalCostmap *costmap_;
  const utils::TrigTable *trig_table_;
};

} // namespace controller
