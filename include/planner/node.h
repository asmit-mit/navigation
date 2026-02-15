#pragma once

namespace planner {

struct Node {
  double x, y, theta;
  double g_cost, h_cost;
  double v, omega;

  int grid_x, grid_y, theta_bin;

  Node *parent;

  Node(double x, double y, double theta);
  Node(double x, double y, double theta, Node *parent);
};

struct CompareNode {
  bool operator()(Node *a, Node *b);
};

}; // namespace planner
