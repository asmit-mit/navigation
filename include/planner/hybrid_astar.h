#include <array>
#include <cmath>
#include <vector>

#include "costmap/costmap.h"
#include "planner/motion_model.h"
#include "planner/optimizer.h"
#include "planner/parameters.h"
#include "planner/pose.h"
#include "planner/state.h"
#include "utils/gen_vector.h"

namespace planner {

class HybridAStar {
public:
  HybridAStar();

  void setParameters(const costmap::Costmap *costmap,
                     const Optimizer *optimizer, MotionModel *motion_mode_,
                     const HybridAstarParams &params);

  void setStart(double x, double y, double theta);
  void setGoal(double x, double y, double theta);
  std::vector<Pose2d> getPlan();

private:
  class Node {
  public:
    Pose3d pose;
    State3d state;

    double g_cost, h_cost;

    Node *parent;
    HybridAStar *planner;

    Node(const Pose3d &p, HybridAStar *planner);
    Node(const Pose3d &p, HybridAStar *planner, Node *parent);

    int getStateIndex() const;
  };

  struct CompareNode {
    bool operator()(Node *a, Node *b);
  };

private:
  State3d poseToState(const Pose3d &p);
  State2d pose2dToState2d(const Pose2d &p);
  Pose2d state2dToPose2d(const State2d &s);

  void buildObstacleCostTable();
  void buildThetaTable();

  void simulate();
  double heuristic(const Node *node);
  bool goalReached(const Node *node);
  std::vector<Pose2d> analyticalExpansion(const Node *node);
  std::vector<std::pair<Pose3d, double>> expand(const Node *node);

  void freeNodes();

private:
  int height_, width_;

  double angular_resolution_, map_resolution_;
  double angular_tolerance_, distance_tolerance_;
  double max_linear_velocity_, max_angular_velocity_;

  double expand_step_;
  double expand_ds_;

  double analytical_expansion_ratio_;
  double analytical_expansion_max_length_;

  double steering_penalty_;
  double change_steering_penalty_;
  double reverse_penalty_;
  double cost_penalty_;
  double expansion_cost_;
  double path_length_weight_;

  int max_explore_iterations_;
  int num_theta_bins_;
  int state_space_size_;

  Pose3d start_, end_;

  static constexpr int dx_[8] = {-1, 1, -1, 0, 1, -1, 0, 1};
  static constexpr int dy_[8] = {0, 0, 1, 1, 1, -1, -1, -1};

  static constexpr int num_samples_ = 5;
  static constexpr double epsilon_ = 1e-6;
  static constexpr double theta_to_deg_ = 180.0 / M_PI;

  const costmap::Costmap *costmap_;
  const Optimizer *optimizer_;
  MotionModel *motion_model_;

  std::vector<double> holonomic_with_obstacle_cost_;

  std::vector<Pose2d> plan_;
  std::vector<Node> node_pool_;

  std::array<std::pair<double, double>, 6> controls_;
  int controls_count_;

  utils::GenVector<double> g_cost_table_;
  utils::GenVector<bool> closed_;

  friend class Node;
};

}; // namespace planner
