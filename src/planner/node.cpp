#include "planner/node.h"

namespace planner {

Node::Node(double x, double y, double theta)
    : x(x), y(y), theta(theta), parent(nullptr) {
  g_cost = 0;
  f_cost = 0;
  h_cost = 0;
}

Node::Node(double x, double y, double theta, Node *parent)
    : x(x), y(y), theta(theta), parent(parent) {
  g_cost = 0;
  f_cost = 0;
  h_cost = 0;
}

bool CompareNode::operator()(Node *a, Node *b) { return a->f_cost > b->f_cost; }

}; // namespace planner
