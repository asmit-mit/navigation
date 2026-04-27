#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/grid_utils.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "controller/parameters.h"
#include "controller/regulated_pure_pursuit.h"
#include "geometry/collison_checker.h"
#include "geometry/pose.h"
#include "grid/global_costmap.h"
#include "grid/local_costmap.h"
#include "grid/parameters.h"
#include "planner/hybrid_astar.h"
#include "planner/motion_model.h"
#include "planner/optimizer.h"
#include "utils/math_utils.h"
#include "utils/trig_utils.h"

using namespace std::placeholders;
using namespace std::chrono_literals;
using std::cout;
using std::endl;

class Navigation : public rclcpp::Node {
public:
  Navigation() : Node("navigation") {
    declareAndGetParameters();

    global_costmap_msg_ = nav_msgs::msg::OccupancyGrid();
    local_costmap_msg_ = nav_msgs::msg::OccupancyGrid();

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map_sub", map_qos, std::bind(&Navigation::mapCallback, this, _1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_sub", 10, std::bind(&Navigation::goalCallback, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom_sub", 10, std::bind(&Navigation::odomCallback, this, _1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path_pub", 10);
    footprint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("footprint_pub", 10);
    global_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap_pub", 10);
    local_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("local_costmap_pub", 10);
    lookahead_pub_ = create_publisher<visualization_msgs::msg::Marker>("lookahead_pub", 10);
    trajectory_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_pub", 10);

    auto local_costmap_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / local_costmap_frequency_));
    auto global_costmap_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / global_costmap_frequency_));
    auto planner_period = std::chrono::milliseconds(static_cast<int>(1000.0 / planner_frequency_));
    auto controller_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / controller_params_.controller_frequency));

    footprint_timer_ = create_wall_timer(local_costmap_period,
                                         std::bind(&Navigation::footprintCallback, this));
    global_costmap_timer_ = create_wall_timer(global_costmap_period,
                                              std::bind(&Navigation::globalCostmapCallback, this));
    local_costmap_timer_ = create_wall_timer(local_costmap_period,
                                             std::bind(&Navigation::localCostmapCallback, this));
    planner_timer_ = create_wall_timer(planner_period,
                                       std::bind(&Navigation::plannerCallback, this));
    controller_timer_ = create_wall_timer(controller_period,
                                          std::bind(&Navigation::controllerCallback, this));

    RCLCPP_INFO(get_logger(), "navigation node started");
  }

private:
  void declareAndGetParameters() {
    declare_parameter("global_costmap.frequency", 1);
    declare_parameter("global_costmap.inflation_radius", 0.55);
    declare_parameter("global_costmap.inscribed_radius", 0.1);
    declare_parameter("global_costmap.scaling_factor", 3.0);

    get_parameter("global_costmap.frequency", global_costmap_frequency_);
    get_parameter("global_costmap.inflation_radius", global_costmap_params_.inflation_radius);
    get_parameter("global_costmap.inscribed_radius", global_costmap_params_.inscribed_radius);
    get_parameter("global_costmap.scaling_factor", global_costmap_params_.scaling_factor);

    declare_parameter("local_costmap.frequency", 5);
    declare_parameter("local_costmap.window_size", 3.0);
    declare_parameter("local_costmap.inflation_radius", 1.0);
    declare_parameter("local_costmap.inscribed_radius", 0.1);
    declare_parameter("local_costmap.scaling_factor", 3.0);

    get_parameter("local_costmap.frequency", local_costmap_frequency_);
    get_parameter("local_costmap.window_size", local_costmap_params_.window_size);
    get_parameter("local_costmap.inflation_radius", local_costmap_params_.inflation_radius);
    get_parameter("local_costmap.inscribed_radius", local_costmap_params_.inscribed_radius);
    get_parameter("local_costmap.scaling_factor", local_costmap_params_.scaling_factor);

    declare_parameter("collision_checker.robot_radius", 0.12);
    get_parameter("collision_checker.robot_radius", robot_radius_);

    declare_parameter("planner.frequency", 10);
    declare_parameter("planner.max_linear_velocity", 1.0);
    declare_parameter("planner.max_angular_velocity", 2.0);
    declare_parameter("planner.angular_resolution", 5.0);
    declare_parameter("planner.angular_tolerance", 0.2);
    declare_parameter("planner.distance_tolerance", 0.2);
    declare_parameter("planner.analytical_expansion_ratio", 3.5);
    declare_parameter("planner.analytical_expansion_max_length", 3.0);
    declare_parameter("planner.steering_penalty", 0.8);
    declare_parameter("planner.change_steering_penalty", 0.3);
    declare_parameter("planner.reverse_penalty", 2.0);
    declare_parameter("planner.cost_penalty", 12.0);
    declare_parameter("planner.expansion_cost", 200.0);
    declare_parameter("planner.path_length_weight", 0.985);
    declare_parameter("planner.max_explore_iterations", 50000);
    declare_parameter("planner.motion_model", "DUBINS");
    declare_parameter("planner.optimizer.iterations", 1000);
    declare_parameter("planner.optimizer.smooth_weight", 0.3);
    declare_parameter("planner.optimizer.data_weight", 0.2);

    get_parameter("planner.frequency", planner_frequency_);
    get_parameter("planner.max_linear_velocity", planner_params_.max_linear_velocity);
    get_parameter("planner.max_angular_velocity", planner_params_.max_angular_velocity);
    get_parameter("planner.angular_resolution", planner_params_.angular_resolution);
    get_parameter("planner.angular_tolerance", planner_params_.angular_tolerance);
    get_parameter("planner.distance_tolerance", planner_params_.distance_tolerance);
    get_parameter("planner.analytical_expansion_ratio", planner_params_.analytical_expansion_ratio);
    get_parameter("planner.analytical_expansion_max_length",
                  planner_params_.analytical_expansion_max_length);
    get_parameter("planner.steering_penalty", planner_params_.steering_penalty);
    get_parameter("planner.change_steering_penalty", planner_params_.change_steering_penalty);
    get_parameter("planner.reverse_penalty", planner_params_.reverse_penalty);
    get_parameter("planner.cost_penalty", planner_params_.cost_penalty);
    get_parameter("planner.expansion_cost", planner_params_.expansion_cost);
    get_parameter("planner.path_length_weight", planner_params_.path_length_weight);
    get_parameter("planner.max_explore_iterations", planner_params_.max_explore_iterations);
    get_parameter("planner.motion_model", motion_model_type_);
    get_parameter("planner.optimizer.iterations", optimizer_iterations_);
    get_parameter("planner.optimizer.smooth_weight", optimizer_smooth_weight_);
    get_parameter("planner.optimizer.data_weight", optimizer_data_weight_);

    declare_parameter("controller.frequency", 20);
    declare_parameter("controller.max_linear_velocity", 1.0);
    declare_parameter("controller.max_angular_velocity", 2.0);
    declare_parameter("controller.max_linear_acceleration", 1.0);
    declare_parameter("controller.max_angular_acceleration", 1.8);
    declare_parameter("controller.sim_time", 1.0);
    declare_parameter("controller.lookahead_distance", 0.6);
    declare_parameter("controller.lookahead_gain", 1.5);
    declare_parameter("controller.max_lookahead_distance", 0.9);
    declare_parameter("controller.min_lookahead_distance", 0.3);
    declare_parameter("controller.proximity_distance", 0.3);
    declare_parameter("controller.proximity_heurisitc_scale", 1.0);
    declare_parameter("controller.approach_velocity_scaling_dist", 1.0);
    declare_parameter("controller.min_approach_linear_velocity", 0.05);
    declare_parameter("controller.min_heading_angle_error", 0.785);
    declare_parameter("controller.distance_tolerance", 0.1);

    get_parameter("controller.frequency", controller_params_.controller_frequency);
    get_parameter("controller.max_linear_velocity", controller_params_.max_linear_velocity);
    get_parameter("controller.max_angular_velocity", controller_params_.max_angular_velocity);
    get_parameter("controller.max_linear_acceleration", controller_params_.max_linear_acceleration);
    get_parameter("controller.max_angular_acceleration",
                  controller_params_.max_angular_acceleration);
    get_parameter("controller.sim_time", controller_params_.sim_time);
    get_parameter("controller.lookahead_distance", controller_params_.lookahead_distance);
    get_parameter("controller.lookahead_gain", controller_params_.lookahead_gain);
    get_parameter("controller.max_lookahead_distance", controller_params_.max_lookahead_distance);
    get_parameter("controller.min_lookahead_distance", controller_params_.min_lookahead_distance);
    get_parameter("controller.proximity_distance", controller_params_.proximity_distance);
    get_parameter("controller.proximity_heuristic_scale",
                  controller_params_.proximity_heuristic_scale);
    get_parameter("controller.approach_velocity_scaling_dist",
                  controller_params_.approach_velocity_scaling_dist);
    get_parameter("controller.min_approach_linear_velocity",
                  controller_params_.min_approach_linear_velocity);
    get_parameter("controller.min_heading_angle_error", controller_params_.min_heading_angle_error);
    get_parameter("controller.distance_tolerance", controller_params_.distance_tolerance);
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = msg;
    have_map_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    start_pose_.pose = msg->pose.pose;
    linear_velocity_ = msg->twist.twist.linear.x;
    angular_velocity_ = msg->twist.twist.angular.z;

    have_start_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose_.pose = msg->pose;

    have_goal_ = true;
  }

  void footprintCallback() {
    if (!have_start_ || !have_map_)
      return;

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = latest_map_->header.frame_id;
    marker.header.stamp = now();
    marker.ns = "robot_footprint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = start_pose_.pose.position.x;
    marker.pose.position.y = start_pose_.pose.position.y;
    marker.pose.position.z = 0.0;

    marker.pose.orientation = start_pose_.pose.orientation;

    marker.scale.x = 0.03;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    const int num_points = 60;
    marker.points.clear();

    for (int i = 0; i <= num_points; ++i) {
      double angle = 2.0 * M_PI * i / num_points;

      geometry_msgs::msg::Point p;
      p.x = robot_radius_ * trig_table_.cos(angle);
      p.y = robot_radius_ * trig_table_.sin(angle);
      p.z = 0.0;

      marker.points.push_back(p);
    }

    visualization_msgs::msg::MarkerArray footprint_marker_arr;
    footprint_marker_arr.markers.push_back(marker);

    footprint_pub_->publish(footprint_marker_arr);
  }

  void globalCostmapCallback() {
    if (!have_map_)
      return;

    edt_.computeDT(latest_map_);
    have_edt_ = true;

    global_costmap_.setParameters(latest_map_,
                                  &edt_,
                                  global_costmap_params_.inflation_radius,
                                  global_costmap_params_.inscribed_radius,
                                  global_costmap_params_.scaling_factor);

    int size = global_costmap_.getHeight() * global_costmap_.getWidth();

    global_costmap_msg_.header.frame_id = latest_map_->header.frame_id;
    global_costmap_msg_.header.stamp = now();
    global_costmap_msg_.info = latest_map_->info;
    global_costmap_msg_.data.resize(size);

    for (int i = 0; i < size; i++) {
      if (latest_map_->data[i] == 100) {
        global_costmap_msg_.data[i] = 100;
      } else {
        double c = global_costmap_.getCostAt(i);
        global_costmap_msg_.data[i] = static_cast<int8_t>(std::min(100.0, c / 2.55));
      }
    }

    // RCLCPP_INFO(get_logger(), "Publishing Global Costmap");
    global_costmap_pub_->publish(global_costmap_msg_);
  }

  void localCostmapCallback() {
    if (!have_start_ || !have_map_ || !have_edt_)
      return;

    double start_x = start_pose_.pose.position.x;
    double start_y = start_pose_.pose.position.y;

    local_costmap_.setParameters(latest_map_,
                                 &edt_,
                                 start_x,
                                 start_y,
                                 local_costmap_params_.window_size,
                                 local_costmap_params_.inflation_radius,
                                 local_costmap_params_.inscribed_radius,
                                 local_costmap_params_.scaling_factor);

    int local_size = local_costmap_.getWindowWidth() * local_costmap_.getWindowHeight();

    local_costmap_msg_.header.frame_id = latest_map_->header.frame_id;
    local_costmap_msg_.header.stamp = now();
    local_costmap_msg_.info = latest_map_->info;
    local_costmap_msg_.info.width = local_costmap_.getWindowWidth();
    local_costmap_msg_.info.height = local_costmap_.getWindowHeight();
    local_costmap_msg_.info.origin.position.x = local_costmap_.getWindowOriginX();
    local_costmap_msg_.info.origin.position.y = local_costmap_.getWindowOriginY();
    local_costmap_msg_.data.resize(local_size);

    for (int i = 0; i < local_size; i++) {
      double c = local_costmap_.getCostAt(i);
      local_costmap_msg_.data[i] = static_cast<int8_t>(std::min(100.0, c / 2.55));
    }

    // RCLCPP_INFO(get_logger(), "Publishing Local Costmap");
    local_costmap_pub_->publish(local_costmap_msg_);

    // if (collision_checker_.inCollisionGlobal(geometry::Pose2d(start_x, start_y)))
    //   RCLCPP_INFO(get_logger(), "In collision global");
    //
    // if (collision_checker_.inCollisionLocal(geometry::Pose2d(start_x, start_y)))
    //   RCLCPP_INFO(get_logger(), "In collision local");
  }

  void plannerCallback() {
    if (!latest_map_ || !have_start_ || !have_goal_)
      return;

    collision_checker_.setParameters(&global_costmap_, &local_costmap_, robot_radius_);

    start_x_ = start_pose_.pose.position.x;
    start_y_ = start_pose_.pose.position.y;
    start_theta_ = tf2::getYaw(start_pose_.pose.orientation);

    goal_x_ = goal_pose_.pose.position.x;
    goal_y_ = goal_pose_.pose.position.y;
    goal_theta_ = tf2::getYaw(goal_pose_.pose.orientation);

    // RCLCPP_INFO(get_logger(), "Start: %f %f", start_x_, start_y_);
    // RCLCPP_INFO(get_logger(), "Goal: %f %f", goal_x_, goal_y_);

    optimizer_.setCostmap(&global_costmap_);
    optimizer_.setIterations(optimizer_iterations_);
    optimizer_.setWeights(optimizer_smooth_weight_, optimizer_data_weight_);

    motion_model_.setTrigTable(&trig_table_);
    if (motion_model_type_ == "DUBINS")
      motion_model_.setMotionModel(planner::MotionModelType::DUBINS);
    else
      motion_model_.setMotionModel(planner::MotionModelType::REED_SHEPPS);
    motion_model_.setDistanceResolution(1.41421356 * global_costmap_.getResolution());
    motion_model_.setMinTurningRadius(planner_params_.max_linear_velocity /
                                      planner_params_.max_angular_velocity);
    motion_model_.setTolerance(planner_params_.distance_tolerance,
                               planner_params_.angular_tolerance);

    planner_.setParameters(&global_costmap_,
                           &optimizer_,
                           &motion_model_,
                           &collision_checker_,
                           &trig_table_,
                           planner_params_);
    planner_.setStart(start_x_, start_y_, start_theta_);
    planner_.setGoal(goal_x_, goal_y_, goal_theta_);

    // auto start = std::chrono::steady_clock::now();

    path_ = planner_.getPlan();

    // auto end = std::chrono::steady_clock::now();

    // double time_ms =
    //     std::chrono::duration<double, std::milli>(end - start).count();

    // RCLCPP_INFO(get_logger(), "Planning time: %f ms", time_ms);

    if (path_.empty()) {
      RCLCPP_WARN(get_logger(), "No path found");
      return;
    }

    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = latest_map_->header.frame_id;
    ros_path.header.stamp = now();

    for (const auto &pose : path_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = ros_path.header;

      pose_stamped.pose.position.x = pose.x;
      pose_stamped.pose.position.y = pose.y;
      pose_stamped.pose.position.z = 0.0;

      ros_path.poses.push_back(pose_stamped);
    }

    path_pub_->publish(ros_path);

    // RCLCPP_INFO(get_logger(), "Path published (%ld poses)",
    //             ros_path.poses.size());
  }

  void controllerCallback() {
    if (!latest_map_ || !have_start_ || !have_goal_)
      return;

    controller_.setParameters(
        &local_costmap_, &collision_checker_, &trig_table_, controller_params_);

    auto start_pose = geometry::Pose3d(start_x_, start_y_, start_theta_);
    auto goal_pose = geometry::Pose3d(goal_x_, goal_y_, goal_theta_);
    auto [v, w] = controller_.computeCommand(
        start_pose, goal_pose, linear_velocity_, angular_velocity_, path_);

    // RCLCPP_INFO(get_logger(), "Distance to goal: %lf", utils::distance(start_pose, goal_pose));
    RCLCPP_INFO(get_logger(), "Publishing v: %lf and w: %lf", v, w);

    geometry::Pose2d lookahead_point = controller_.getLookaheadPoint();

    visualization_msgs::msg::Marker lookahead_marker;

    lookahead_marker.header.frame_id = latest_map_->header.frame_id;
    lookahead_marker.header.stamp = get_clock()->now();

    lookahead_marker.ns = "lookahead";
    lookahead_marker.id = 0;

    lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
    lookahead_marker.action = visualization_msgs::msg::Marker::ADD;

    lookahead_marker.pose.position.x = lookahead_point.x;
    lookahead_marker.pose.position.y = lookahead_point.y;
    lookahead_marker.pose.position.z = 0.0;

    lookahead_marker.scale.x = 0.05;
    lookahead_marker.scale.y = 0.05;
    lookahead_marker.scale.z = 0.05;

    lookahead_marker.color.r = 1.0;
    lookahead_marker.color.g = 0.0;
    lookahead_marker.color.b = 0.5;
    lookahead_marker.color.a = 1.0;

    lookahead_marker.lifetime = rclcpp::Duration::from_seconds(0);

    lookahead_pub_->publish(lookahead_marker);

    std::vector<visualization_msgs::msg::Marker> trajectory_markers;

    double sim_x = start_x_;
    double sim_y = start_y_;
    double sim_theta = start_theta_;

    double dt = 1.0 / controller_params_.controller_frequency;

    double sim_time = 0.0;
    int i = 0;

    while (sim_time < controller_params_.sim_time) {
      sim_x += v * trig_table_.cos(sim_theta) * dt;
      sim_y += v * trig_table_.sin(sim_theta) * dt;
      sim_theta += w * dt;

      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = latest_map_->header.frame_id;
      marker.header.stamp = get_clock()->now();

      marker.ns = "trajectory";
      marker.id = i++;

      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = sim_x;
      marker.pose.position.y = sim_y;
      marker.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, sim_theta);
      marker.pose.orientation = tf2::toMsg(q);

      marker.scale.x = 0.3;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      double t = sim_time / controller_params_.sim_time;

      marker.color.r = 1.0 - t;
      marker.color.g = t;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.lifetime = rclcpp::Duration::from_seconds(0);
      trajectory_markers.push_back(marker);
      sim_time += dt;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers = trajectory_markers;

    trajectory_pub_->publish(marker_array);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;

    cmd_vel_pub_->publish(cmd);
  }

private:
  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr global_costmap_timer_;
  rclcpp::TimerBase::SharedPtr local_costmap_timer_;
  rclcpp::TimerBase::SharedPtr footprint_timer_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid global_costmap_msg_;
  nav_msgs::msg::OccupancyGrid local_costmap_msg_;

  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  std::vector<geometry::Pose2d> path_;

  utils::EDT edt_;
  utils::TrigTable trig_table_ = utils::TrigTable(10000);
  grid::GlobalCostmap global_costmap_;
  grid::LocalCostmap local_costmap_;
  planner::HybridAStar planner_;
  planner::Optimizer optimizer_;
  planner::MotionModel motion_model_;
  geometry::CollisionChecker collision_checker_;
  controller::RegulatedPurePursuit controller_;

  controller::ControllerParams controller_params_;
  planner::HybridAstarParams planner_params_;
  grid::CostmapParams global_costmap_params_;
  grid::CostmapParams local_costmap_params_;
  double robot_radius_;
  std::string motion_model_type_;
  int optimizer_iterations_;
  double optimizer_smooth_weight_, optimizer_data_weight_;

  double start_x_, start_y_, start_theta_;
  double goal_x_, goal_y_, goal_theta_;

  bool have_start_ = false;
  bool have_goal_ = false;
  bool have_map_ = false;
  bool have_edt_ = false;

  int global_costmap_frequency_, local_costmap_frequency_, planner_frequency_;

  double linear_velocity_, angular_velocity_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation>());
  rclcpp::shutdown();
}
