#include <array>
#include <vector>

#include "costmap/costmap.h"
#include "planner/motion_model.h"
#include "planner/optimizer.h"
#include "planner/pose.h"
#include "planner/state.h"

namespace planner {

class HybridAStar {
public:
  HybridAStar();

  void setMotionModel(MotionModelType type);
  void setCostmap(const costmap::Costmap *costmap);
  void setTolerance(double angle, double distance);
  void setResolutions(double distance, double angle);
  void setVelocities(double linear, double angluar);
  void setStart(double x, double y, double theta);
  void setGoal(double x, double y, double theta);
  void setIterations(int iterations);
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

  struct NodeHash {
    std::size_t operator()(Node *node) const;
  };

  struct NodeEqual {
    bool operator()(Node *a, Node *b) const;
  };

private:
  State3d poseToState(const Pose3d &p);
  State2d pose2dToState2d(const Pose2d &p);
  Pose2d state2dToPose2d(const State2d &s);

  void preprocess();
  void simulate();
  double heuristic(const Node *node);
  bool goalReached(const Node *node);
  std::vector<Pose2d> analyticalExpansion(const Node *node);
  std::vector<std::pair<Pose3d, double>> expand(const Node *node);

  void freeNodes();

private:
  double map_resolution_;
  int height_, width_;

  double angular_resolution_, distance_resolution_;
  double angular_tolerance_, distance_tolerance_;
  double max_linear_velocity_, max_angular_velocity_;

  double expand_step_;
  double expand_ds_;

  Pose3d start_, end_;

  static constexpr int num_samples_ = 5;

  static constexpr int dx_[8] = {-1, 1, -1, 0, 1, -1, 0, 1};
  static constexpr int dy_[8] = {0, 0, 1, 1, 1, -1, -1, -1};

  static constexpr double steering_penalty_ = 1.7;
  static constexpr double change_steering_penalty_ = 0.1;
  static constexpr double reverse_penalty_ = 2.0;
  static constexpr double cost_penalty_ = 15.0;
  static constexpr double expansion_cost_ = 200.0;
  static constexpr double path_length_weight_ = 0.985;
  static constexpr double epsilon_ = 1e-6;

  const costmap::Costmap *costmap_;
  MotionModel motion_model_;
  Optimizer optimizer_;

  std::vector<double> holonomic_with_obstacle_cost_;
  std::vector<Pose2d> plan_;

  std::vector<Node *> nodes_;
  std::array<std::pair<double, double>, 6> controls_;

  friend class Node;
};

}; // namespace planner
