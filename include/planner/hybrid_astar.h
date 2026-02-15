#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <queue>
#include <unordered_set>
#include <vector>

#include "planner/node.h"
#include "planner/pose.h"
#include "planner/state.h"
#include "planner/reed_shepps.h"
#include "voronoi/VoronoiImage.h"

namespace planner {

class HybridAStar {
public:
  HybridAStar(nav_msgs::msg::OccupancyGrid::SharedPtr grid);

  void setStart(double x, double y, double theta);
  void setGoal(double x, double y, double theta);
  std::vector<Pose> getPlan();

private:
  std::pair<int, int> worldToMapDiscrete(double x, double y);
  std::pair<double, double> worldToMapContinous(double x, double y);
  std::pair<double, double> mapToWorld(double x, double y);

  bool isValid(int x, int y);
  int getIndex(int x, int y);
  void preprocess();
  double distance(const Pose &a, const Pose &b);
  double heuristic(const Node *a);

private:
  double resolution_;
  int height_, width_;
  Pose start_, end_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  voronoi::VoronoiImage voronoi_;
  std::priority_queue<Node *, std::vector<Node *>, planner::CompareNode> open;
  std::unordered_set<State, StateHash> closed;
  ReedShepps reed_shepps_;

  std::vector<double> holonomic_with_obstacle_cost;
};

}; // namespace planner
