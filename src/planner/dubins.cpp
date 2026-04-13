#include "planner/dubins.h"
#include "planner/motion_states.h"
#include "utils/math_utils.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace planner {

Dubins::Dubins() {
  min_turning_radius_ = 1.0;
  distance_resolution_ = 0.05;
  angular_tolerance_ = 0.1;
  distance_tolerance_ = 0.1;
}

void Dubins::setTrigTable(const utils::TrigTable *trig_table) {
  trig_table_ = trig_table;
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

void Dubins::simulate(const geometry::Pose3d &start, const geometry::Pose3d &end) {
  assert(min_turning_radius_ > 0);
  assert(distance_resolution_ > 0);
  assert(trig_table_ != nullptr);

  assert(std::isfinite(start.x) && std::isfinite(start.y) &&
         std::isfinite(start.theta));
  assert(std::isfinite(end.x) && std::isfinite(end.y) &&
         std::isfinite(end.theta));

  start_ = start;
  end_ = end;

  if (utils::distance(start, end) < distance_tolerance_) {
    optimal_path_dist_ = 0.0;
    return;
  }

  optimal_path_dist_ = std::numeric_limits<double>::infinity();
  optimal_path_segment_count_ = 0;

  geometry::Pose3d p = utils::changeOfBasis(start, end);
  p.x /= min_turning_radius_;
  p.y /= min_turning_radius_;

  geometry::Pose3d p_reflect(p.x, -p.y, -p.theta);

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

std::vector<geometry::Pose2d> Dubins::getOptimalPath() {
  std::vector<geometry::Pose2d> poses;
  geometry::Pose3d curr = start_;
  poses.push_back(curr);

  for (int i = 0; i < optimal_path_segment_count_; i++) {
    const auto &segment = optimal_path_[i];

    double curvature = 0.0;
    if (segment.steering == Steering::LEFT)
      curvature = 1.0 / min_turning_radius_;
    else if (segment.steering == Steering::RIGHT)
      curvature = -1.0 / min_turning_radius_;

    double remaining = std::abs(segment.param) * min_turning_radius_;

    while (remaining > 1e-9) {
      double ds = std::min(distance_resolution_, remaining);
      double dtheta = ds * curvature;

      if (std::abs(curvature) < 1e-9) {
        curr.x += ds * trig_table_->cos(curr.theta);
        curr.y += ds * trig_table_->sin(curr.theta);
      } else {
        double R = 1.0 / curvature;
        curr.x += R * (trig_table_->sin(curr.theta + dtheta) -
                       trig_table_->sin(curr.theta));
        curr.y -= R * (trig_table_->cos(curr.theta + dtheta) -
                       trig_table_->cos(curr.theta));
        curr.theta += dtheta;
      }

      curr.theta = utils::M(curr.theta);
      poses.push_back(curr);

      remaining -= ds;
    }
  }

  return poses;
}

void Dubins::tryPath(double dist, int count, bool reflect) {
  if (!std::isfinite(dist) || dist >= optimal_path_dist_)
    return;

  for (int i = 0; i < count; i++) {
    if (reflect)
      candidate_path_[i].reverseSteering();
  }

  optimal_path_dist_ = dist;
  optimal_path_segment_count_ = count;
  std::copy_n(candidate_path_.begin(), count, optimal_path_.begin());
}

void Dubins::computeParams(const geometry::Pose3d &p, double &alpha, double &beta,
                           double &d) {
  double dx = p.x;
  double dy = p.y;

  d = std::hypot(dx, dy);
  double theta = (d > 1e-9) ? std::atan2(dy, dx) : 0.0;

  alpha = utils::M2pi(-theta);
  beta = utils::M2pi(p.theta - theta);
}

void Dubins::pathLSL(const geometry::Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = trig_table_->sin(alpha), sb = trig_table_->sin(beta);
  double ca = trig_table_->cos(alpha), cb = trig_table_->cos(beta);
  double c_ab = trig_table_->cos(alpha - beta);

  double tmp0 = d + sa - sb;
  double p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb);

  if (p_sq < 0)
    return;

  double tmp1 = atan2(cb - ca, tmp0);

  double t = utils::M2pi(tmp1 - alpha);
  double u = sqrt(p_sq);
  double v = utils::M2pi(beta - tmp1);

  double dist = t + u + v;

  candidate_path_[0] = PathElement(t, Steering::LEFT);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT);
  candidate_path_[2] = PathElement(v, Steering::LEFT);

  tryPath(dist, 3, reflect);
}

void Dubins::pathRSR(const geometry::Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = trig_table_->sin(alpha), sb = trig_table_->sin(beta);
  double ca = trig_table_->cos(alpha), cb = trig_table_->cos(beta);
  double c_ab = trig_table_->cos(alpha - beta);

  double tmp0 = d - sa + sb;
  double p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa);

  if (p_sq < 0)
    return;

  double tmp1 = atan2(ca - cb, tmp0);

  double t = utils::M2pi(alpha - tmp1);
  double u = sqrt(p_sq);
  double v = utils::M2pi(tmp1 - beta);

  double dist = t + u + v;

  candidate_path_[0] = PathElement(t, Steering::RIGHT);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT);
  candidate_path_[2] = PathElement(v, Steering::RIGHT);

  tryPath(dist, 3, reflect);
}

void Dubins::pathLSR(const geometry::Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = trig_table_->sin(alpha), sb = trig_table_->sin(beta);
  double ca = trig_table_->cos(alpha), cb = trig_table_->cos(beta);
  double c_ab = trig_table_->cos(alpha - beta);

  double p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb);
  if (p_sq < 0)
    return;

  double u = sqrt(p_sq);

  double tmp0 = atan2(-ca - cb, d + sa + sb) - atan2(-2.0, u);

  double t = utils::M2pi(tmp0 - alpha);
  double v = utils::M2pi(tmp0 - beta);

  double dist = t + u + v;

  candidate_path_[0] = PathElement(t, Steering::LEFT);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT);
  candidate_path_[2] = PathElement(v, Steering::RIGHT);

  tryPath(dist, 3, reflect);
}

void Dubins::pathRSL(const geometry::Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = trig_table_->sin(alpha), sb = trig_table_->sin(beta);
  double ca = trig_table_->cos(alpha), cb = trig_table_->cos(beta);
  double c_ab = trig_table_->cos(alpha - beta);

  double p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb);
  if (p_sq < 0)
    return;

  double u = sqrt(p_sq);

  double tmp0 = atan2(ca + cb, d - sa - sb) - atan2(2.0, u);

  double t = utils::M2pi(alpha - tmp0);
  double v = utils::M2pi(beta - tmp0);

  double dist = t + u + v;

  candidate_path_[0] = PathElement(t, Steering::RIGHT);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT);
  candidate_path_[2] = PathElement(v, Steering::LEFT);

  tryPath(dist, 3, reflect);
}

void Dubins::pathRLR(const geometry::Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = trig_table_->sin(alpha), sb = trig_table_->sin(beta);
  double ca = trig_table_->cos(alpha), cb = trig_table_->cos(beta);
  double c_ab = trig_table_->cos(alpha - beta);

  double tmp0 = (6 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0;

  if (std::abs(tmp0) > 1)
    return;

  double p_val = utils::M2pi(2 * M_PI - acos(tmp0));

  double phi = atan2(ca - cb, d - sa + sb);

  double t = utils::M2pi(alpha - phi + p_val / 2);
  double u = p_val;
  double v = utils::M2pi(alpha - beta - t + p_val);

  double dist = t + u + v;

  candidate_path_[0] = PathElement(t, Steering::RIGHT);
  candidate_path_[1] = PathElement(u, Steering::LEFT);
  candidate_path_[2] = PathElement(v, Steering::RIGHT);

  tryPath(dist, 3, reflect);
}

void Dubins::pathLRL(const geometry::Pose3d &p, bool reflect) {
  double alpha, beta, d;
  computeParams(p, alpha, beta, d);

  double sa = trig_table_->sin(alpha), sb = trig_table_->sin(beta);
  double ca = trig_table_->cos(alpha), cb = trig_table_->cos(beta);
  double c_ab = trig_table_->cos(alpha - beta);

  double tmp0 = (6 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0;

  if (std::abs(tmp0) > 1)
    return;

  double p_val = utils::M2pi(2 * M_PI - acos(tmp0));

  double phi = atan2(ca - cb, d + sa - sb);

  double t = utils::M2pi(-alpha - phi + p_val / 2);
  double u = p_val;
  double v = utils::M2pi(beta - alpha - t + p_val);

  double dist = t + u + v;

  candidate_path_[0] = PathElement(t, Steering::LEFT);
  candidate_path_[1] = PathElement(u, Steering::RIGHT);
  candidate_path_[2] = PathElement(v, Steering::LEFT);

  tryPath(dist, 3, reflect);
}

} // namespace planner
