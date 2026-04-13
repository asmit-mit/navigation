#include "planner/reed_shepps.h"
#include "utils/math_utils.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <limits>

namespace planner {

ReedShepps::ReedShepps() {
  min_turning_radius_ = 1.0;
  distance_resolution_ = 0.05;
  angular_tolerance_ = 0.1;
  distance_tolerance_ = 0.1;
}

void ReedShepps::setTrigTable(const utils::TrigTable *trig_table) {
  trig_table_ = trig_table;
}

void ReedShepps::setDistanceResolution(double resolution) {
  distance_resolution_ = resolution;
}

void ReedShepps::setTolerance(double angle, double distance) {
  angular_tolerance_ = angle;
  distance_tolerance_ = distance;
}

void ReedShepps::setMinTurningRadius(double min_radius) {
  min_turning_radius_ = min_radius;
}

void ReedShepps::simulate(const geometry::Pose3d &start,
                          const geometry::Pose3d &end) {
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
    optimal_path_dist_ = 0;
    return;
  }

  optimal_path_dist_ = std::numeric_limits<double>::infinity();
  optimal_path_segment_count_ = 0;

  geometry::Pose3d relative = utils::changeOfBasis(start, end);
  relative.x /= min_turning_radius_;
  relative.y /= min_turning_radius_;

  double x = relative.x, y = relative.y, theta = relative.theta;

  geometry::Pose3d relative_timeflip = geometry::Pose3d(-x, y, -theta);
  geometry::Pose3d relative_reflect = geometry::Pose3d(x, -y, -theta);
  geometry::Pose3d relative_timeflip_reflect = geometry::Pose3d(-x, -y, theta);

  for (auto fn : pathFns) {
    (this->*fn)(relative, false, false);
    (this->*fn)(relative_timeflip, true, false);
    (this->*fn)(relative_reflect, false, true);
    (this->*fn)(relative_timeflip_reflect, true, true);
  }
}

double ReedShepps::getOptimalDistance() {
  return optimal_path_dist_ * min_turning_radius_;
}

std::vector<geometry::Pose2d> ReedShepps::getOptimalPath() {
  std::vector<geometry::Pose2d> poses;
  geometry::Pose3d curr = start_;
  poses.push_back(curr);

  for (int i = 0; i < optimal_path_segment_count_; i++) {
    const auto &segment = optimal_path_[i];

    double direction = (segment.gear == Gear::FORWARD) ? 1.0 : -1.0;
    double curvature = 0.0;

    if (segment.steering == Steering::LEFT)
      curvature = 1.0 / min_turning_radius_;
    else if (segment.steering == Steering::RIGHT)
      curvature = -1.0 / min_turning_radius_;

    double remaining = std::abs(segment.param) * min_turning_radius_;

    while (remaining > 1e-9) {
      double ds = std::min(distance_resolution_, remaining);
      double d = direction * ds;
      double dtheta = d * curvature;

      if (std::abs(curvature) < 1e-9) {
        curr.x += d * trig_table_->cos(curr.theta);
        curr.y += d * trig_table_->sin(curr.theta);
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

void ReedShepps::tryPath(double dist, int count, bool timeflip, bool reflect) {
  if (!std::isfinite(dist) || dist >= optimal_path_dist_)
    return;

  for (int i = 0; i < count; i++) {
    if (reflect)
      candidate_path_[i].reverseSteering();

    if (timeflip)
      candidate_path_[i].reverseGear();
  }

  optimal_path_dist_ = dist;
  optimal_path_segment_count_ = count;
  std::copy_n(candidate_path_.begin(), count, optimal_path_.begin());
}

void ReedShepps::path1(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  auto [u, t] =
      utils::R(p.x - trig_table_->sin(phi), p.y - 1 + trig_table_->cos(phi));
  double v = utils::M(phi - t);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT, Gear::FORWARD);
  candidate_path_[2] = PathElement(v, Steering::LEFT, Gear::FORWARD);

  tryPath(dist, 3, timeflip, reflect);
}

void ReedShepps::path2(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = utils::M(p.theta);
  auto [rho, t1] =
      utils::R(p.x + trig_table_->sin(phi), p.y - 1 - trig_table_->cos(phi));

  if (rho * rho < 4)
    return;

  double u = std::sqrt(rho * rho - 4);
  double t = utils::M(t1 + std::atan2(2, u));
  double v = utils::M(t - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT, Gear::FORWARD);
  candidate_path_[2] = PathElement(v, Steering::RIGHT, Gear::FORWARD);

  tryPath(dist, 3, timeflip, reflect);
}

void ReedShepps::path3(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x - trig_table_->sin(phi);
  double eta = p.y - 1 + trig_table_->cos(phi);
  auto [rho, theta] = utils::R(x1, eta);

  if (rho > 4)
    return;

  double A = acos(std::clamp(rho / 4, -1.0, 1.0));
  double t = utils::M(theta + M_PI_2 + A);
  double u = utils::M(M_PI - 2 * A);
  double v = utils::M(phi - t - u);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::RIGHT, Gear::BACKWARD);
  candidate_path_[2] = PathElement(v, Steering::LEFT, Gear::FORWARD);

  tryPath(dist, 3, timeflip, reflect);
}

void ReedShepps::path4(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x - trig_table_->sin(phi);
  double eta = p.y - 1 + trig_table_->cos(phi);
  auto [rho, theta] = utils::R(x1, eta);

  if (rho > 4)
    return;

  double A = acos(std::clamp(rho / 4, -1.0, 1.0));
  double t = utils::M(theta + M_PI_2 + A);
  double u = utils::M(M_PI - 2 * A);
  double v = utils::M(t + u - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::RIGHT, Gear::BACKWARD);
  candidate_path_[2] = PathElement(v, Steering::LEFT, Gear::BACKWARD);

  tryPath(dist, 3, timeflip, reflect);
}

void ReedShepps::path5(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x - trig_table_->sin(phi);
  double eta = p.y - 1 + trig_table_->cos(phi);
  auto [rho, theta] = utils::R(x1, eta);

  if (rho > 4)
    return;

  double u = acos(std::clamp(1 - rho * rho / 8, -1.0, 1.0));
  double A = asin(2 * trig_table_->sin(u) / rho);
  double t = utils::M(theta + M_PI_2 - A);
  double v = utils::M(t - u - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::RIGHT, Gear::FORWARD);
  candidate_path_[2] = PathElement(v, Steering::LEFT, Gear::BACKWARD);

  tryPath(dist, 3, timeflip, reflect);
}

void ReedShepps::path6(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x + trig_table_->sin(phi);
  double eta = p.y - 1 - trig_table_->cos(phi);
  auto [rho, theta] = utils::R(x1, eta);

  if (rho > 4)
    return;

  double A, t, u, v;
  if (rho <= 2) {
    A = acos(std::clamp((rho + 2) / 4, -1.0, 1.0));
    t = utils::M(theta + M_PI_2 + A);
    u = utils::M(A);
    v = utils::M(phi - t + 2 * u);
  } else {
    A = acos(std::clamp((rho - 2) / 4, -1.0, 1.0));
    t = utils::M(theta + M_PI_2 - A);
    u = utils::M(M_PI - A);
    v = utils::M(phi - t + 2 * u);
  }

  double dist = std::abs(t) + std::abs(u) + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::RIGHT, Gear::FORWARD);
  candidate_path_[2] = PathElement(u, Steering::LEFT, Gear::BACKWARD);
  candidate_path_[3] = PathElement(v, Steering::RIGHT, Gear::BACKWARD);

  tryPath(dist, 4, timeflip, reflect);
}

void ReedShepps::path7(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x + trig_table_->sin(phi);
  double eta = p.y - 1 - trig_table_->cos(phi);
  auto [rho, theta] = utils::R(xi, eta);
  double u1 = (20 - rho * rho) / 16;

  if (rho > 6)
    return;

  if (u1 < 0 || u1 > 1)
    return;

  double u = acos(u1);
  double A = asin(2 * trig_table_->sin(u) / rho);
  double t = utils::M(theta + M_PI_2 + A);
  double v = utils::M(t - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::RIGHT, Gear::BACKWARD);
  candidate_path_[2] = PathElement(u, Steering::LEFT, Gear::BACKWARD);
  candidate_path_[3] = PathElement(v, Steering::RIGHT, Gear::FORWARD);

  tryPath(dist, 4, timeflip, reflect);
}

void ReedShepps::path8(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x - trig_table_->sin(phi);
  double eta = p.y - 1 + trig_table_->cos(phi);
  auto [rho, theta] = utils::R(xi, eta);

  if (rho < 2)
    return;

  double u = std::sqrt(rho * rho - 4) - 2;
  double A = std::atan2(2, u + 2);
  double t = utils::M(theta + M_PI_2 + A);
  double v = utils::M(t - phi + M_PI_2);

  double dist = std::abs(t) + M_PI_2 + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(M_PI_2, Steering::RIGHT, Gear::BACKWARD);
  candidate_path_[2] = PathElement(u, Steering::STRAIGHT, Gear::BACKWARD);
  candidate_path_[3] = PathElement(v, Steering::LEFT, Gear::BACKWARD);

  tryPath(dist, 4, timeflip, reflect);
}

void ReedShepps::path9(const geometry::Pose3d &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x - trig_table_->sin(phi);
  double eta = p.y - 1 + trig_table_->cos(phi);
  auto [rho, theta] = utils::R(xi, eta);

  if (rho < 2)
    return;

  double u = std::sqrt(rho * rho - 4) - 2;
  double A = std::atan2(u + 2, 2);
  double t = utils::M(theta + M_PI_2 - A);
  double v = utils::M(t - phi - M_PI_2);

  double dist = std::abs(t) + std::abs(u) + M_PI_2 + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT, Gear::FORWARD);
  candidate_path_[2] = PathElement(M_PI_2, Steering::RIGHT, Gear::FORWARD);
  candidate_path_[3] = PathElement(v, Steering::LEFT, Gear::BACKWARD);

  tryPath(dist, 4, timeflip, reflect);
}

void ReedShepps::path10(const geometry::Pose3d &p, bool timeflip,
                        bool reflect) {
  double phi = p.theta;
  double xi = p.x + trig_table_->sin(phi);
  double eta = p.y - 1 - trig_table_->cos(phi);
  auto [rho, theta] = utils::R(xi, eta);

  if (rho < 2)
    return;

  double t = utils::M(theta + M_PI_2);
  double u = rho - 2;
  double v = utils::M(phi - t - M_PI_2);

  double dist = std::abs(t) + M_PI_2 + std::abs(u) + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(M_PI_2, Steering::RIGHT, Gear::BACKWARD);
  candidate_path_[2] = PathElement(u, Steering::STRAIGHT, Gear::BACKWARD);
  candidate_path_[3] = PathElement(v, Steering::RIGHT, Gear::BACKWARD);

  tryPath(dist, 4, timeflip, reflect);
}

void ReedShepps::path11(const geometry::Pose3d &p, bool timeflip,
                        bool reflect) {
  double phi = p.theta;
  double xi = p.x + trig_table_->sin(phi);
  double eta = p.y - 1 - trig_table_->cos(phi);

  auto [rho, theta] = utils::R(xi, eta);

  if (rho < 2)
    return;

  double t = utils::M(theta);
  double u = rho - 2;
  double v = utils::M(phi - t - M_PI_2);

  double dist = std::abs(t) + std::abs(u) + M_PI_2 + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(u, Steering::STRAIGHT, Gear::FORWARD);
  candidate_path_[2] = PathElement(M_PI_2, Steering::LEFT, Gear::FORWARD);
  candidate_path_[3] = PathElement(v, Steering::RIGHT, Gear::BACKWARD);

  tryPath(dist, 4, timeflip, reflect);
}

void ReedShepps::path12(const geometry::Pose3d &p, bool timeflip,
                        bool reflect) {
  double phi = p.theta;
  double xi = p.x + trig_table_->sin(phi);
  double eta = p.y - 1 - trig_table_->cos(phi);

  auto [rho, theta] = utils::R(xi, eta);

  if (rho < 4)
    return;

  double u = std::sqrt(rho * rho - 4) - 4;
  double A = std::atan2(2, u + 4);
  double t = utils::M(theta + M_PI_2 + A);
  double v = utils::M(t - phi);

  double dist = std::abs(t) + M_PI_2 + std::abs(u) + M_PI_2 + std::abs(v);

  candidate_path_[0] = PathElement(t, Steering::LEFT, Gear::FORWARD);
  candidate_path_[1] = PathElement(M_PI_2, Steering::RIGHT, Gear::BACKWARD);
  candidate_path_[2] = PathElement(u, Steering::STRAIGHT, Gear::BACKWARD);
  candidate_path_[3] = PathElement(M_PI_2, Steering::LEFT, Gear::BACKWARD);
  candidate_path_[4] = PathElement(v, Steering::RIGHT, Gear::FORWARD);

  tryPath(dist, 5, timeflip, reflect);
}

}; // namespace planner
