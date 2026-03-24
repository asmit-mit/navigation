#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <vector>

#include "planner/grid_map.h"
#include "planner/motion_model.h"
#include "planner/optimizer.h"
#include "planner/pose.h"
#include "planner/state.h"

namespace planner {

class HybridAStar {
public:
  HybridAStar();

  void setMotionModel(MotionModelType type);
  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);
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

  void preprocess();
  void simulate();
  double heuristic(const Node *node);
  bool goalReached(const Node *node);
  std::vector<Pose3d> analyticalExpansion(const Node *node);
  std::vector<std::pair<Pose3d, double>> expand(const Node *node);

  void freeNodes();

private:
  double map_resolution_;
  int height_, width_;

  double angular_resolution_, distance_resolution_;
  double angular_tolerance_, distance_tolerance_;
  double max_linear_velocity_, max_angular_velocity_;

  Pose3d start_, end_;

  static constexpr int num_samples_ = 5;

  static constexpr int dx_[8] = {-1, 1, -1, 0, 1, -1, 0, 1};
  static constexpr int dy_[8] = {0, 0, 1, 1, 1, -1, -1, -1};

  static constexpr double penalty_steering_ = 1.05;
  static constexpr double penalty_change_steering_ = 1.5;
  static constexpr double penalty_reverse_ = 3.0;

  MotionModel motion_model_;
  GridMap grid_;
  Optimizer optimizer_;

  std::vector<double> holonomic_with_obstacle_cost_;
  std::vector<Pose2d> plan_;

  std::vector<Node *> nodes_;
  std::vector<std::pair<double, double>> controls_;

  friend class Node;
};

}; // namespace planner
