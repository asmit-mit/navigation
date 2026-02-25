#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <vector>

#include "planner/pose.h"
#include "planner/reed_shepps.h"
#include "planner/state.h"
#include "voronoi/VoronoiImage.h"

namespace planner {

class HybridAStar {
public:
  HybridAStar();

  void setGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid);
  void setTolerance(double angle, double distance);
  void setResolutions(double distance, double angle);
  void setVelocities(double linear, double angluar);
  void setStart(double x, double y, double theta);
  void setGoal(double x, double y, double theta);
  void setIterations(int iterations);
  std::vector<Pose3d> getPlan();

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

  struct Grad {
    double x, y;

    Grad();
    Grad(double x, double y);

    Grad operator*(double scalar) const;
    Grad operator+(const Grad &other) const;
  };

private:
  std::pair<int, int> worldToMapDiscrete(double x, double y);
  std::pair<double, double> worldToMapContinous(double x, double y);
  std::pair<double, double> mapToWorld(double x, double y);

  State3d poseToState(const Pose3d &p);
  
  State2d pose2dToState2d(const Pose2d &p);
  Pose2d state2dToPose2d(const State2d &s);

  bool isValid(int x, int y);
  int getIndex(int x, int y);
  void preprocess();
  void simulate();
  void smoothen();
  double heuristic(const Node *node);
  bool goalReached(const Node *node);
  std::vector<Pose3d> analyticalExpansion(const Node *node);
  std::vector<std::pair<Pose3d, double>> expand(const Node *node);
  double optimizationStep(double w_rho, double w_o, double w_kappa, double w_s,
                          double alpha, double dmax, double kappa_max);
  
  double voronoiCost(int idx, double w_rho, double alpha);
  double obstacleCost(int idx, double w_o, double dmax);
  double curvatureCost(int idx, double w_kappa, double kappa_max);
  double smoothnessCost(int idx, double w_s);

  void freeNodes();

private:
  double map_resolution_;
  double angular_resolution_, distance_resolution_;
  double angular_tolerance_, distance_tolerance_;
  double max_linear_velocity_, max_angular_velocity_;
  int iterations_;

  int height_, width_;
  Pose3d start_, end_;

  static constexpr int num_samples_ = 5;
  static constexpr double step_ = 0.01;

  static constexpr double w_rho_ = 0.05;
  static constexpr double w_o_ = 0.05;
  static constexpr double w_kapa_ = 0.01;
  static constexpr double w_s_ = 0.2;

  static constexpr double alpha_ = 0.1;
  static constexpr double dmax_ = 3.0;
  static constexpr double kappa_max_ = 0.01;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  voronoi::VoronoiImage voronoi_;
  ReedShepps reed_shepps_;
  std::vector<double> holonomic_with_obstacle_cost_;
  std::vector<Pose3d> plan_;
  std::vector<Grad> grad_;

  std::vector<Node *> nodes_;
  std::vector<std::pair<double, double>> controls_;

  friend class Node;
};

}; // namespace planner
