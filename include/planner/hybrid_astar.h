#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <queue>
#include <unordered_set>
#include <vector>

#include "planner/pose.h"
#include "planner/reed_shepps.h"
#include "planner/state.h"
#include "voronoi/VoronoiImage.h"

namespace planner {

class HybridAStar {
public:
  HybridAStar(nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  void setTolerance(double angle, double distance);
  void setAngularResolution(double angle);
  void setVelocities(double linear, double angluar);
  void setStart(double x, double y, double theta);
  void setGoal(double x, double y, double theta);
  std::vector<Pose> getPlan();

private:
  class Node {
  public:
    Pose pose;
    State state;

    double g_cost, h_cost;

    Node *parent;
    HybridAStar *planner;

    Node(const Pose &p, HybridAStar *planner);
    Node(const Pose &p, HybridAStar *planner, Node *parent);
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
  std::pair<int, int> worldToMapDiscrete(double x, double y);
  std::pair<double, double> worldToMapContinous(double x, double y);
  std::pair<double, double> mapToWorld(double x, double y);

  State poseToState(const Pose &p);

  bool isValid(int x, int y);
  int getIndex(int x, int y);
  void preprocess();
  double distance(const Pose &a, const Pose &b);
  double heuristic(const Node *a);
  bool goalReached(const Node *node);
  std::vector<std::pair<Pose, double>> expand(const Node *p);

  void freeNodes();

private:
  double angular_resolution_, distance_resolution_;
  double angular_tolerance_, distance_tolerance_;
  double max_linear_velocity_, max_angular_velocity_;

  int height_, width_;
  Pose start_, end_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  voronoi::VoronoiImage voronoi_;
  ReedShepps reed_shepps_;
  std::vector<double> holonomic_with_obstacle_cost;

  std::vector<Node *> nodes;

  friend class Node;
};

}; // namespace planner
