// made using https://github.com/nathanlct/reeds-shepp-curves/

#include "planner/reed_shepps.h"

#include <algorithm>
#include <cstdlib>
#include <limits>

namespace planner {

double ReedShepps::M(double theta) {
  theta = std::fmod(theta, 2 * M_PI);

  if (theta < 0)
    theta += 2 * M_PI;

  if (theta >= M_PI)
    theta -= 2 * M_PI;

  return theta;
}

void ReedShepps::setMinTurningRadius(double min_turning_radius_) {
  this->min_turning_radius_ = min_turning_radius_;
}

double ReedShepps::getOptimalPath(const Pose &start, const Pose &end) {
  double min_dist = std::numeric_limits<double>::infinity();

  Pose relative = changeOfBasis(start, end);
  double x = relative.x, y = relative.y, theta = relative.theta;

  Pose relative_timeflip = Pose(-x, y, -theta);
  Pose relative_reflect = Pose(x, -y, -theta);
  Pose relative_timeflip_reflect = Pose(-x, -y, theta);

  for (auto fn : pathFns) {
    min_dist = std::min(min_dist, (this->*fn)(relative));
    min_dist = std::min(min_dist, (this->*fn)(relative_timeflip));
    min_dist = std::min(min_dist, (this->*fn)(relative_reflect));
    min_dist = std::min(min_dist, (this->*fn)(relative_timeflip_reflect));
  }

  return min_dist * min_turning_radius_;
}

std::pair<double, double> ReedShepps::R(double x, double y) {
  double r = std::hypot(x, y);
  double theta = std::atan2(y, x);
  return {r, theta};
}

Pose ReedShepps::changeOfBasis(const Pose &p1, const Pose &p2) {
  double theta1 = p1.theta;
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  double new_x = dx * std::cos(theta1) + dy * std::sin(theta1);
  double new_y = -dx * std::sin(theta1) + dy * std::cos(theta1);
  double new_theta = p2.theta - p1.theta;

  return Pose(new_x, new_y, new_theta);
}

double ReedShepps::rad2deg(double rad) { return 180 * rad / M_PI; }

double ReedShepps::deg2rad(double deg) { return M_PI * deg / 180; }

int ReedShepps::sign(int x) { return (x >= 0) ? 1 : -1; }

double ReedShepps::getAllPaths() { return 0; }

double ReedShepps::path1(const Pose &p) {
  double phi = p.theta;
  auto [u, t] = R(p.x - std::sin(phi), p.y - 1 + std::cos(phi));
  double v = M(phi - t);

  return std::abs(t) + std::abs(u) + std::abs(v);
}

double ReedShepps::path2(const Pose &p) {
  double phi = M(p.theta);
  auto [rho, t1] = R(p.x + std::sin(phi), p.y - 1 - std::cos(phi));

  if (rho * rho < 4)
    return std::numeric_limits<double>::infinity();

  double u = std::sqrt(rho * rho - 4);
  double t = M(t1 + std::atan2(2, u));
  double v = M(t - phi);

  return std::abs(t) + std::abs(u) + std::abs(v);
}

double ReedShepps::path3(const Pose &p) {
  double phi = p.theta;
  double x1 = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return std::numeric_limits<double>::infinity();

  double A = std::acos(rho / 4);
  double t = M(theta + M_PI_2 + A);
  double u = M(M_PI - 2 * A);
  double v = M(phi - t - u);

  return std::abs(t) + std::abs(u) + std::abs(v);
}

double ReedShepps::path4(const Pose &p) {
  double phi = p.theta;
  double x1 = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return std::numeric_limits<double>::infinity();

  double A = std::acos(rho / 4);
  double t = M(theta + M_PI_2 + A);
  double u = M(M_PI - 2 * A);
  double v = M(t + u - phi);

  return std::abs(t) + std::abs(u) + std::abs(v);
}

double ReedShepps::path5(const Pose &p) {
  double phi = p.theta;
  double x1 = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return std::numeric_limits<double>::infinity();

  double u = std::acos(1 - rho * rho / 8);
  double A = std::asin(2 * std::sin(u) / rho);
  double t = M(theta + M_PI_2 - A);
  double v = M(t - u - phi);

  return std::abs(t) + std::abs(u) + std::abs(v);
}

double ReedShepps::path6(const Pose &p) {
  double phi = p.theta;
  double x1 = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return std::numeric_limits<double>::infinity();

  double A, t, u, v;
  if (rho <= 2) {
    A = std::acos((rho + 2) / 4);
    t = M(theta + M_PI_2 + A);
    u = M(A);
    v = M(phi - t + 2 * u);
  } else {
    A = std::acos((rho - 2) / 4);
    t = M(theta + M_PI_2 - A);
    u = M(M_PI - A);
    v = M(phi - t + 2 * u);
  }

  return std::abs(t) + std::abs(u) + std::abs(u) + std::abs(v);
}

double ReedShepps::path7(const Pose &p) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);
  auto [rho, theta] = R(xi, eta);
  double u1 = (20 - rho * rho) / 16;

  if (rho <= 6 && u1 >= 0 && u1 <= 1) {
    double u = std::acos(u1);
    double A = std::asin(2 * std::sin(u) / rho);
    double t = M(theta + M_PI_2 + A);
    double v = M(t - phi);

    return std::abs(t) + std::abs(u) + std::abs(u) + std::abs(v);
  }

  return std::numeric_limits<double>::infinity();
}

double ReedShepps::path8(const Pose &p) {
  double phi = p.theta;
  double xi = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return std::numeric_limits<double>::infinity();

  double u = std::sqrt(rho * rho - 4) - 2;
  double A = std::atan2(2, u + 2);
  double t = M(theta + M_PI_2 + A);
  double v = M(t - phi + M_PI_2);

  return std::abs(t) + M_PI_2 + std::abs(u) + std::abs(v);
}

double ReedShepps::path9(const Pose &p) {
  double phi = p.theta;
  double xi = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);

  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return std::numeric_limits<double>::infinity();

  double u = std::sqrt(rho * rho - 4) - 2;
  double A = std::atan2(u + 2, 2);
  double t = M(theta + M_PI_2 - A);
  double v = M(t - phi - M_PI_2);

  return std::abs(t) + std::abs(u) + M_PI_2 + std::abs(v);
}

double ReedShepps::path10(const Pose &p) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);
  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return std::numeric_limits<double>::infinity();

  double t = M(theta + M_PI_2);
  double u = rho - 2;
  double v = M(phi - t - M_PI_2);

  return std::abs(t) + M_PI_2 + std::abs(u) + std::abs(v);
}

double ReedShepps::path11(const Pose &p) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);

  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return std::numeric_limits<double>::infinity();

  double t = M(theta);
  double u = rho - 2;
  double v = M(phi - t - M_PI_2);

  return std::abs(t) + std::abs(u) + M_PI_2 + std::abs(v);
}

double ReedShepps::path12(const Pose &p) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);

  auto [rho, theta] = R(xi, eta);

  if (rho < 4)
    return std::numeric_limits<double>::infinity();

  double u = std::sqrt(rho * rho - 4) - 4;
  double A = std::atan2(2, u + 4);
  double t = M(theta + M_PI_2 + A);
  double v = M(t - phi);

  return std::abs(t) + M_PI_2 + std::abs(u) + M_PI_2 + std::abs(v);
}
}; // namespace planner
