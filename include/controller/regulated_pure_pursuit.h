#include <vector>

#include "controller/parameters.h"
#include "geometry/pose.h"
#include "grid/local_costmap.h"
#include "utils/trig_utils.h"

namespace contoller {

class RegulatedPurePursuit {
public:
  RegulatedPurePursuit();

  void setParameters(const grid::LocalCostmap *costmap,
                     const utils::TrigTable *trig_table,
                     controller::ControllerParams &params);

  std::pair<double, double>
  computeCommand(const geometry::Pose2d &current_pose,
                 const std::vector<geometry::Pose2d> &plan);

private:
  geometry::Pose2d findLookaheadPoint();
  geometry::Pose2d transformToLocalFrame();
  double computeCurvature();

  std::pair<double, double> computeVelocity();

  double regulateByCurvature(double v, double curvature);
  double regulateByCostmap(const geometry::Pose2d &, double v);

  bool isGoalReached();
  double computeLookaheadDistance(double speed);

  const grid::LocalCostmap *costmap_;
  const utils::TrigTable *trig_table_;
  controller::ControllerParams params_;
};

} // namespace contoller
