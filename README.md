# Minimal Nav Stack

A lightweight ROS 2 navigation stack built from scratch as a personal learning project, inspired by [Nav2](https://nav2.ros.org/).
This project focuses on understanding, implementing and experimenting with path planning and motion planning algorithms under real-world constraints.

---

![Demo](./media/motion-planning.gif)

---

## Overview

This project implements a minimal but fully functional navigation stack comprising three core components:

- **Costmap** — Built using Euclidean Distance Transform (EDT) for efficient obstacle inflation and smooth cost gradients.
- **Planner** — Hybrid A\* for kinematically feasible path planning that respects vehicle constraints.
- **Controller** — Regulated Pure Pursuit for smooth, speed-adaptive path tracking.

## Results

- Achieved navigation speeds of up to **1.0 m/s** in TurtleBot3 simulation.
- Hybrid A\* planner benchmarked across maps ranging from **200×200 to 600×400** cells, consistently achieving planning times of **< 100 ms**

## Parameters

```yaml
navigation:
  ros__parameters:

    global_costmap:
      frequency: 1                          # Global costmap update frequency
      inflation_radius: 0.55                # Distance obstacles are "expanded" for safety buffer (meters)
      inscribed_radius: 0.11                # Robot's inner radius (used for collision checks) (meters)
      scaling_factor: 3.0                   # How quickly cost decays from obstacles (higher = sharper drop)

    local_costmap:
      frequency: 5
      window_size: 3.0                      # Size of local rolling window around robot (meters)
      inflation_radius: 1.0
      inscribed_radius: 0.11
      scaling_factor: 3.0
    
    collision_checker:
      robot_radius: 0.12                    # robot radius used for collision checks (meters)

    planner:
      frequency: 5                          # Path Planning frequency
      max_linear_velocity: 1.0              # Max speed used for planning (not actual execution limit) (m/s)
      max_angular_velocity: 2.0             # Max turning rate for planning (rad/s)
      angular_resolution: 5.0               # Degrees per discretized heading (lower = more precise but slower)
      angular_tolerance: 0.2                # Acceptable orientation error at goal (radians)
      distance_tolerance: 0.2               # Acceptable positional error at goal (meters)
      analytical_expansion_ratio: 3.5       # How often to attempt shortcut (Reeds-Shepp/Dubins) (lower = more attempts for shortcut)
      analytical_expansion_max_length: 3.0  # Max length of shortcut connection (meters)
      expansion_cost: 200.0                 # Maximum cost threshold allowed in shortcut path
      steering_penalty: 1.3                 # Penalizes turning (prefers straight motion) (should be >= 1.0)
      change_steering_penalty: 1.8          # Penalizes switching left to right turns (reduces zig-zag in path) (should be >= 1.0)
      reverse_penalty: 2.1                  # Penalizes reversing (higher = fewer cusps) (should be >= 1.0)
      cost_penalty: 2.0                     # Weight for costmap values (obstacle proximity influence)
      path_length_weight: 0.985             # Weight for path length vs other costs (closer to 1 = shorter paths) (should be <= 1.0)
      max_explore_iterations: 50000         # Hard cap on A* expansions (prevents infinite search)
      motion_model: "DUBINS"                # "DUBINS": no reversing | "REED_SHEPPS": reversing allowed
      optimizer:
        iterations: 1000                    # Number of smoothing iterations
        smooth_weight: 0.3                  # Weight for smoothness (higher = smoother, less accurate)
        data_weight: 0.2                    # Weight for matching original

    controller:
      frequency: 10                         # Control loop frequency
      max_linear_velocity: 1.0              # Max forward speed (m/s)
      max_angular_velocity: 2.0             # Max turning speed (rad/s)
      max_linear_acceleration: 0.8          # Linear acceleration (m/s2)
      max_angular_acceleration: 2.3         # Angular acceleration (rad/s2)
      sim_time: 1.0                         # Time horizon (in seconds) for collision checking
      lookahead_distance: 0.6               # Base lookahead distance (meters)
      lookahead_gain: 1.5                   # Scale to change lookahead distance with speed
      max_lookahead_distance: 0.9           # Upper bound for lookeahed distance
      min_lookahead_distance: 0.3           # Lower bound for lookeahed distance
      proximity_distance: 0.3               # Distance from obstacle to start slowing down (meters)
      proximity_heuristic_scale: 1.0        # Strength of obstacle slowdown effect
      approach_velocity_scaling_dist: 1.0   # Distance from goal to start slowing down
      min_approach_linear_velocity: 0.05    # Minimum crawl speed near goal (m/s)
      min_heading_angle_error: 0.785        # ~45°. If error > this → rotate in place (radians)
      distance_tolerance: 0.1               # When goal is considered reached (meters)
```

## References

### Papers

Dubins, L. E. (1957). *On curves of minimal length with a constraint on average curvature, and with prescribed initial and terminal positions and tangents.* American Journal of Mathematics, 79(3), 497–516.
https://www.jstor.org/stable/2372560


Reeds, J. A., & Shepp, L. A. (1990). *Optimal paths for a car that goes both forwards and backwards.* Pacific Journal of Mathematics, 145(2), 367–393.
https://projecteuclid.org/euclid.pjm/1102645450


Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2008). *Practical search techniques in path planning for autonomous driving.*
https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf


Felzenszwalb, P. F., & Huttenlocher, D. P. (2012). *Distance Transforms of Sampled Functions.* Theory of Computing, 8(1), 415–428.
https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf


Macenski, S., Singh, S., Martín, F., & Ginés, J. (2023). *Regulated Pure Pursuit for Robot Path Tracking.* Autonomous Robots (Springer).
https://arxiv.org/abs/2305.20026


Ohnishi, F., & Takahashi, M. (2026). *Dynamic Window Pure Pursuit Considering Velocity and Acceleration Constraints.*
https://arxiv.org/abs/2601.15006

---

### Code

Reeds-Shepp curves: adapted from nathanlct/reeds-shepp-curves (MIT License)
https://github.com/nathanlct/reeds-shepp-curves

---
