#include "planner/optimizer.h"

#include <cmath>

namespace planner {

Optimizer::Optimizer() {}

Optimizer::Grad::Grad() {
  x = 0;
  y = 0;
}

Optimizer::Grad::Grad(double x, double y) : x(x), y(y) {}

Optimizer::Grad Optimizer::Grad::operator*(double scalar) const {
  return Grad(x * scalar, y * scalar);
}

Optimizer::Grad Optimizer::Grad::operator+(const Grad &other) const {
  return Grad(x + other.x, y + other.y);
}

void Optimizer::setIterations(int iterations) { iterations_ = iterations; }

void Optimizer::setStepSize(double step_size) { step_ = step_size; }

std::vector<Pose2d> Optimizer::getSmoothPath(std::vector<Pose2d> &plan) {
  const int n = plan.size();
  grad_.resize(n);

  int it = 0;
  while (it < iterations_) {
    it++;

    for (Grad &g : grad_) {
      g.x = 0;
      g.y = 0;
    }

    for (int i = 1; i < n - 1; i++) {
      // do costs here
    }

    grad_[0] = Grad(0, 0);
    grad_[n - 1] = Grad(0, 0);

    for (int j = 1; j < n - 1; j++) {
      double norm = std::hypot(grad_[j].x, grad_[j].y);
      double max_grad = 5.0;

      if (norm > max_grad) {
        grad_[j].x *= max_grad / norm;
        grad_[j].y *= max_grad / norm;
      }

      plan[j].x -= step_ * grad_[j].x;
      plan[j].y -= step_ * grad_[j].y;
    }
  }

  return plan;
}

}; // namespace planner
