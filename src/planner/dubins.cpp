#include "planner/dubins.h"
#include "utils/math_utils.h"

#include <algorithm>
#include <limits>

namespace planner {

Dubins::Dubins() {
  min_turning_radius_ = 1.0;
  distance_resolution_ = 0.05;
  angular_tolerance_ = 0.1;
  distance_tolerance_ = 0.1;
  optimal_path_dist_ = std::numeric_limits<double>::infinity();
}

void Dubins::setDistanceResolution(double resolution) {
  distance_resolution_ = resolution;
}

void Dubins::setTolerance(double angle, double distance) {
  angular_tolerance_ = angle;
  distance_tolerance_ = distance;
}

void Dubins::setMinTurningRadius(double min_radius) {
  min_turning_radius_ = min_radius;
}

void Dubins::simulate(const Pose3d &start, const Pose3d &end) {
  start_ = start;
  end_ = end;

  if (utils::distance(start, end) < distance_tolerance_) {
    optimal_path_.clear();
    optimal_path_dist_ = 0.0;
    return;
  }

  optimal_path_dist_ = std::numeric_limits<double>::infinity();
  optimal_path_.clear();

  Pose3d p = utils::changeOfBasis(start, end);
  p.x /= min_turning_radius_;
  p.y /= min_turning_radius_;

  Pose3d p_reflect(p.x, -p.y, -p.theta);

  pathLSL(p, false);
  pathLSL(p_reflect, true);

  pathRSR(p, false);
  pathRSR(p_reflect, true);

  pathLSR(p, false);
  pathLSR(p_reflect, true);

  pathRSL(p, false);
  pathRSL(p_reflect, true);

  pathRLR(p, false);
  pathRLR(p_reflect, true);

  pathLRL(p, false);
  pathLRL(p_reflect, true);
}

double Dubins::getOptimalDistance() {
  return optimal_path_dist_ * min_turning_radius_;
}

std::vector<Pose3d> Dubins::getOptimalPath() {
  std::vector<Pose3d> poses;
  Pose3d curr = start_;
  poses.push_back(curr);

  for (const auto &segment : optimal_path_) {
    double curvature = 0.0;

    if (segment.steering == Steering::LEFT)
      curvature = 1.0 / min_turning_radius_;
    else if (segment.steering == Steering::RIGHT)
      curvature = -1.0 / min_turning_radius_;

    double remaining = std::abs(segment.param) * min_turning_radius_;
    double step = distance_resolution_;

    while (remaining > 1e-9) {
      double ds = std::min(step, remaining);
      double dtheta = ds * curvature;

      if (std::abs(curvature) < 1e-9) {
        curr.x += ds * std::cos(curr.theta);
        curr.y += ds * std::sin(curr.theta);
      } else {
        double R = 1.0 / curvature;
        curr.x += R * (std::sin(curr.theta + dtheta) - std::sin(curr.theta));
        curr.y -= R * (std::cos(curr.theta + dtheta) - std::cos(curr.theta));
        curr.theta += dtheta;
      }

      curr.theta = utils::M(curr.theta);
      poses.push_back(curr);

      remaining -= ds;
    }
  }

  return poses;
}

void Dubins::tryPath(double dist, std::vector<PathElement> candidate,
                     bool reflect) {
  if (!std::isfinite(dist) || dist >= optimal_path_dist_)
    return;

  if (reflect) {
    for (auto &e : candidate)
      e.reverseSteering();
  }

  optimal_path_dist_ = dist;
  optimal_path_ = std::move(candidate);
}

void Dubins::computeParams(const Pose3d &p, double &alpha, double &beta,
                           double &d) {
  double dx = p.x;
  double dy = p.y;

  d = std::hypot(dx, dy);
  double theta = (d > 1e-9) ? std::atan2(dy, dx) : 0.0;

  alpha = utils::mod2pi(-theta);
  beta = utils::mod2pi(p.theta - theta);
}

void Dubins::pathLSL(const Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = sin(alpha), sb = sin(beta);
  double ca = cos(alpha), cb = cos(beta);
  double c_ab = cos(alpha - beta);

  double tmp0 = d + sa - sb;
  double p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb);

  if (p_sq < 0)
    return;

  double tmp1 = atan2(cb - ca, tmp0);

  double t = utils::mod2pi(tmp1 - alpha);
  double u = sqrt(p_sq);
  double v = utils::mod2pi(beta - tmp1);

  double dist = t + u + v;

  std::vector<PathElement> path = {
      {t, Steering::LEFT}, {u, Steering::STRAIGHT}, {v, Steering::LEFT}};

  tryPath(dist, std::move(path), reflect);
}

void Dubins::pathRSR(const Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = sin(alpha), sb = sin(beta);
  double ca = cos(alpha), cb = cos(beta);
  double c_ab = cos(alpha - beta);

  double tmp0 = d - sa + sb;
  double p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa);

  if (p_sq < 0)
    return;

  double tmp1 = atan2(ca - cb, tmp0);

  double t = utils::mod2pi(alpha - tmp1);
  double u = sqrt(p_sq);
  double v = utils::mod2pi(tmp1 - beta);

  double dist = t + u + v;

  std::vector<PathElement> path = {
      {t, Steering::RIGHT}, {u, Steering::STRAIGHT}, {v, Steering::RIGHT}};

  tryPath(dist, std::move(path), reflect);
}

void Dubins::pathLSR(const Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = sin(alpha), sb = sin(beta);
  double ca = cos(alpha), cb = cos(beta);
  double c_ab = cos(alpha - beta);

  double p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb);
  if (p_sq < 0)
    return;

  double u = sqrt(p_sq);

  double tmp0 = atan2(-ca - cb, d + sa + sb) - atan2(-2.0, u);

  double t = utils::mod2pi(tmp0 - alpha);
  double v = utils::mod2pi(tmp0 - beta);

  double dist = t + u + v;

  std::vector<PathElement> path = {
      {t, Steering::LEFT}, {u, Steering::STRAIGHT}, {v, Steering::RIGHT}};

  tryPath(dist, std::move(path), reflect);
}

void Dubins::pathRSL(const Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = sin(alpha), sb = sin(beta);
  double ca = cos(alpha), cb = cos(beta);
  double c_ab = cos(alpha - beta);

  double p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb);
  if (p_sq < 0)
    return;

  double u = sqrt(p_sq);

  double tmp0 = atan2(ca + cb, d - sa - sb) - atan2(2.0, u);

  double t = utils::mod2pi(alpha - tmp0);
  double v = utils::mod2pi(beta - tmp0);

  double dist = t + u + v;

  std::vector<PathElement> path = {
      {t, Steering::RIGHT}, {u, Steering::STRAIGHT}, {v, Steering::LEFT}};

  tryPath(dist, std::move(path), reflect);
}

void Dubins::pathRLR(const Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = sin(alpha), sb = sin(beta);
  double ca = cos(alpha), cb = cos(beta);
  double c_ab = cos(alpha - beta);

  double tmp0 = (6 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0;

  if (std::abs(tmp0) > 1)
    return;

  double p_val = utils::mod2pi(2 * M_PI - acos(tmp0));

  double phi = atan2(ca - cb, d - sa + sb);

  double t = utils::mod2pi(alpha - phi + p_val / 2);
  double u = p_val;
  double v = utils::mod2pi(alpha - beta - t + p_val);

  double dist = t + u + v;

  std::vector<PathElement> path = {
      {t, Steering::RIGHT}, {u, Steering::LEFT}, {v, Steering::RIGHT}};

  tryPath(dist, std::move(path), reflect);
}

void Dubins::pathLRL(const Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = sin(alpha), sb = sin(beta);
  double ca = cos(alpha), cb = cos(beta);
  double c_ab = cos(alpha - beta);

  double tmp0 = (6 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0;

  if (std::abs(tmp0) > 1)
    return;

  double p_val = utils::mod2pi(2 * M_PI - acos(tmp0));

  double phi = atan2(ca - cb, d + sa - sb);

  double t = utils::mod2pi(-alpha - phi + p_val / 2);
  double u = p_val;
  double v = utils::mod2pi(beta - alpha - t + p_val);

  double dist = t + u + v;

  std::vector<PathElement> path = {
      {t, Steering::LEFT}, {u, Steering::RIGHT}, {v, Steering::LEFT}};

  tryPath(dist, std::move(path), reflect);
}

} // namespace planner
