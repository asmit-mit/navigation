#include "planner/reed_shepps.h"

#include <cstdlib>
#include <limits>

namespace planner {

ReedShepps::ReedShepps() {}

double ReedShepps::M(double theta) {
  theta = std::fmod(theta, 2 * M_PI);

  if (theta < 0)
    theta += 2 * M_PI;

  if (theta >= M_PI)
    theta -= 2 * M_PI;

  return theta;
}

void ReedShepps::setDistanceResolution(double resolution) {
  distance_resolution_ = resolution;
}

void ReedShepps::setMinTurningRadius(double min_radius) {
  min_turning_radius_ = min_radius;
}

void ReedShepps::simulate(const Pose &start, const Pose &end) {
  start_ = start;
  end_ = end;

  optimal_path_dist_ = std::numeric_limits<double>::infinity();
  optimal_path_.clear();

  Pose relative = changeOfBasis(start, end);
  relative.x /= min_turning_radius_;
  relative.y /= min_turning_radius_;

  double x = relative.x, y = relative.y, theta = relative.theta;

  Pose relative_timeflip = Pose(-x, y, -theta);
  Pose relative_reflect = Pose(x, -y, -theta);
  Pose relative_timeflip_reflect = Pose(-x, -y, theta);

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

std::vector<Pose> ReedShepps::getOptimalPath() {
  std::vector<Pose> poses;
  Pose curr = start_;
  poses.push_back(curr);

  for (const auto &segment : optimal_path_) {
    double direction = (segment.gear == Gear::FORWARD) ? 1.0 : -1.0;
    double curvature = 0.0;

    if (segment.steering == Steering::LEFT)
      curvature = 1.0 / min_turning_radius_;
    else if (segment.steering == Steering::RIGHT)
      curvature = -1.0 / min_turning_radius_;

    double remaining = std::abs(segment.param) * min_turning_radius_;
    double step = distance_resolution_;

    while (remaining > 1e-9) {
      double ds = std::min(step, remaining);
      double d = direction * ds;
      double dtheta = d * curvature;

      if (std::abs(curvature) < 1e-9) {
        curr.x += d * std::cos(curr.theta);
        curr.y += d * std::sin(curr.theta);
      } else {
        double R = 1.0 / curvature;
        curr.x += R * (std::sin(curr.theta + dtheta) - std::sin(curr.theta));
        curr.y -= R * (std::cos(curr.theta + dtheta) - std::cos(curr.theta));
        curr.theta += dtheta;
      }

      curr.theta = M(curr.theta);
      poses.push_back(curr);

      remaining -= ds;
    }
  }

  return poses;
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

void ReedShepps::path1(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  auto [u, t] = R(p.x - std::sin(phi), p.y - 1 + std::cos(phi));
  double v = M(phi - t);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::STRAIGHT, Gear::FORWARD));
  path.emplace_back(PathElement(v, Steering::LEFT, Gear::FORWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path2(const Pose &p, bool timeflip, bool reflect) {
  double phi = M(p.theta);
  auto [rho, t1] = R(p.x + std::sin(phi), p.y - 1 - std::cos(phi));

  if (rho * rho < 4)
    return;

  double u = std::sqrt(rho * rho - 4);
  double t = M(t1 + std::atan2(2, u));
  double v = M(t - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::STRAIGHT, Gear::FORWARD));
  path.emplace_back(PathElement(v, Steering::RIGHT, Gear::FORWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path3(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return;

  double A = std::acos(rho / 4);
  double t = M(theta + M_PI_2 + A);
  double u = M(M_PI - 2 * A);
  double v = M(phi - t - u);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::RIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(v, Steering::LEFT, Gear::FORWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path4(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return;

  double A = std::acos(rho / 4);
  double t = M(theta + M_PI_2 + A);
  double u = M(M_PI - 2 * A);
  double v = M(t + u - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::RIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(v, Steering::LEFT, Gear::BACKWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path5(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return;

  double u = std::acos(1 - rho * rho / 8);
  double A = std::asin(2 * std::sin(u) / rho);
  double t = M(theta + M_PI_2 - A);
  double v = M(t - u - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::RIGHT, Gear::FORWARD));
  path.emplace_back(PathElement(v, Steering::LEFT, Gear::BACKWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path6(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double x1 = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);
  auto [rho, theta] = R(x1, eta);

  if (rho > 4)
    return;

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

  double dist = std::abs(t) + std::abs(u) + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::RIGHT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::LEFT, Gear::BACKWARD));
  path.emplace_back(PathElement(v, Steering::RIGHT, Gear::BACKWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path7(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);
  auto [rho, theta] = R(xi, eta);
  double u1 = (20 - rho * rho) / 16;

  if (rho > 6)
    return;

  if (u1 < 0 || u1 > 1)
    return;

  double u = std::acos(u1);
  double A = std::asin(2 * std::sin(u) / rho);
  double t = M(theta + M_PI_2 + A);
  double v = M(t - phi);

  double dist = std::abs(t) + std::abs(u) + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::RIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(u, Steering::LEFT, Gear::BACKWARD));
  path.emplace_back(PathElement(v, Steering::RIGHT, Gear::FORWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path8(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);
  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return;

  double u = std::sqrt(rho * rho - 4) - 2;
  double A = std::atan2(2, u + 2);
  double t = M(theta + M_PI_2 + A);
  double v = M(t - phi + M_PI_2);

  double dist = std::abs(t) + M_PI_2 + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(M_PI_2, Steering::RIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(u, Steering::STRAIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(v, Steering::LEFT, Gear::BACKWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path9(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x - std::sin(phi);
  double eta = p.y - 1 + std::cos(phi);

  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return;

  double u = std::sqrt(rho * rho - 4) - 2;
  double A = std::atan2(u + 2, 2);
  double t = M(theta + M_PI_2 - A);
  double v = M(t - phi - M_PI_2);

  double dist = std::abs(t) + std::abs(u) + M_PI_2 + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::STRAIGHT, Gear::FORWARD));
  path.emplace_back(PathElement(M_PI_2, Steering::RIGHT, Gear::FORWARD));
  path.emplace_back(PathElement(v, Steering::LEFT, Gear::BACKWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path10(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);
  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return;

  double t = M(theta + M_PI_2);
  double u = rho - 2;
  double v = M(phi - t - M_PI_2);

  double dist = std::abs(t) + M_PI_2 + std::abs(u) + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(M_PI_2, Steering::RIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(u, Steering::STRAIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(v, Steering::RIGHT, Gear::BACKWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path11(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);

  auto [rho, theta] = R(xi, eta);

  if (rho < 2)
    return;

  double t = M(theta);
  double u = rho - 2;
  double v = M(phi - t - M_PI_2);

  double dist = std::abs(t) + std::abs(u) + M_PI_2 + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(u, Steering::STRAIGHT, Gear::FORWARD));
  path.emplace_back(PathElement(M_PI_2, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(v, Steering::RIGHT, Gear::BACKWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

void ReedShepps::path12(const Pose &p, bool timeflip, bool reflect) {
  double phi = p.theta;
  double xi = p.x + std::sin(phi);
  double eta = p.y - 1 - std::cos(phi);

  auto [rho, theta] = R(xi, eta);

  if (rho < 4)
    return;

  double u = std::sqrt(rho * rho - 4) - 4;
  double A = std::atan2(2, u + 4);
  double t = M(theta + M_PI_2 + A);
  double v = M(t - phi);

  double dist = std::abs(t) + M_PI_2 + std::abs(u) + M_PI_2 + std::abs(v);

  std::vector<PathElement> path;
  path.emplace_back(PathElement(t, Steering::LEFT, Gear::FORWARD));
  path.emplace_back(PathElement(M_PI_2, Steering::RIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(u, Steering::STRAIGHT, Gear::BACKWARD));
  path.emplace_back(PathElement(M_PI_2, Steering::LEFT, Gear::BACKWARD));
  path.emplace_back(PathElement(v, Steering::RIGHT, Gear::FORWARD));

  if (timeflip) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseGear();
  }

  if (reflect) {
    for (int i = 0; i < (int)path.size(); i++)
      path[i].reverseSteering();
  }

  if (dist < optimal_path_dist_) {
    optimal_path_dist_ = dist;
    optimal_path_ = path;
  }
}

}; // namespace planner
