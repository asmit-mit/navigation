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
    global_costmap_msg_ = nav_msgs::msg::OccupancyGrid();
    local_costmap_msg_ = nav_msgs::msg::OccupancyGrid();

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos, std::bind(&Navigation::mapCallback, this, _1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&Navigation::goalCallback, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Navigation::odomCallback, this, _1));

    cmd_vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    path_pub_ =
        create_publisher<nav_msgs::msg::Path>("/hybrid_astar_path", 10);

    global_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap", 10);

    local_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap", 10);

    lookahead_pose_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(
            "/lookahead_pose", 10);

    planner_timer_ = create_wall_timer(
        200ms, std::bind(&Navigation::plannerCallback, this));

    controller_timer_ = create_wall_timer(
        100ms, std::bind(&Navigation::controllerCallback, this));

    costmap_timer_ = create_wall_timer(
        200ms, std::bind(&Navigation::costmapCallback, this));

    RCLCPP_INFO(get_logger(), "navigation node started");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = msg;
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

  void costmapCallback() {
    if (!latest_map_)
      return;

    edt_.computeDT(latest_map_);

    global_costmap_.setParameters(latest_map_, &edt_, 0.55, 0.1, 3.0);

    int size = global_costmap_.getHeight() * global_costmap_.getWidth();

    global_costmap_msg_.header = latest_map_->header;
    global_costmap_msg_.info = latest_map_->info;
    global_costmap_msg_.data.resize(size);

    for (int i = 0; i < size; i++) {
      if (latest_map_->data[i] == 100) {
        global_costmap_msg_.data[i] = 100;
      } else {
        double c = global_costmap_.getCostAt(i);
        global_costmap_msg_.data[i] =
            static_cast<int8_t>(std::min(100.0, c / 2.55));
      }
    }

    // RCLCPP_INFO(get_logger(), "Publishing Global Costmap");
    global_costmap_pub_->publish(global_costmap_msg_);

    if (!have_start_)
      return;

    double start_x = start_pose_.pose.position.x;
    double start_y = start_pose_.pose.position.y;

    local_costmap_.setParameters(latest_map_, &edt_, start_x, start_y, 3.0, 1.0,
                                 0.1, 3.0);

    int local_size =
        local_costmap_.getWindowWidth() * local_costmap_.getWindowHeight();

    local_costmap_msg_.header = latest_map_->header;
    local_costmap_msg_.info = latest_map_->info;
    local_costmap_msg_.info.width = local_costmap_.getWindowWidth();
    local_costmap_msg_.info.height = local_costmap_.getWindowHeight();
    local_costmap_msg_.info.origin.position.x =
        local_costmap_.getWindowOriginX();
    local_costmap_msg_.info.origin.position.y =
        local_costmap_.getWindowOriginY();
    local_costmap_msg_.data.resize(local_size);

    for (int i = 0; i < local_size; i++) {
      double c = local_costmap_.getCostAt(i);
      local_costmap_msg_.data[i] =
          static_cast<int8_t>(std::min(100.0, c / 2.55));
    }

    // RCLCPP_INFO(get_logger(), "Publishing Local Costmap");
    local_costmap_pub_->publish(local_costmap_msg_);

    collision_checker_.setParameters(&global_costmap_, &local_costmap_, 0.12);

    // RCLCPP_INFO(get_logger(), "Distance to obstacle: %lf",
    //             local_costmap_.getDistanceAtWorld(start_x, start_y));

    // if (collision_checker_.inCollisionGlobal(
    //         geometry::Pose2d(start_x, start_y)))
    //   RCLCPP_INFO(get_logger(), "In collision global map");
    //
    // if (collision_checker_.inCollisionLocal(
    //         geometry::Pose2d(start_x, start_y)))
    //   RCLCPP_INFO(get_logger(), "In collision local map");
  }

  void plannerCallback() {
    if (!latest_map_ || !have_start_ || !have_goal_)
      return;

    start_x_ = start_pose_.pose.position.x;
    start_y_ = start_pose_.pose.position.y;
    start_theta_ = tf2::getYaw(start_pose_.pose.orientation);

    goal_x_ = goal_pose_.pose.position.x;
    goal_y_ = goal_pose_.pose.position.y;
    goal_theta_ = tf2::getYaw(goal_pose_.pose.orientation);

    // RCLCPP_INFO(get_logger(), "Start: %f %f", start_x_, start_y_);
    // RCLCPP_INFO(get_logger(), "Goal: %f %f", goal_x_, goal_y_);

    optimizer_.setCostmap(&global_costmap_);
    optimizer_.setIterations(1000);
    optimizer_.setWeights(0.3, 0.2);

    motion_model_.setTrigTable(&trig_table_);
    motion_model_.setMotionModel(planner::MotionModelType::DUBINS);
    motion_model_.setDistanceResolution(1.41421356 *
                                        global_costmap_.getResolution());
    motion_model_.setMinTurningRadius(0.5 / 1.0);
    motion_model_.setTolerance(0.5, 0.2);

    planner::HybridAstarParams planner_params;
    planner_params.max_linear_velocity = 0.8;
    planner_params.max_angular_velocity = 2.0;
    planner_params.distance_tolerance = 0.2;
    planner_params.angular_tolerance = 0.2;
    planner_params.angular_resolution = 5;
    planner_params.reverse_penalty = 2.1;
    planner_params.steering_penalty = 0.8;
    planner_params.change_steering_penalty = 0.3;
    planner_params.cost_penalty = 12.0;

    planner_.setParameters(&global_costmap_, &optimizer_, &motion_model_,
                           &collision_checker_, &trig_table_, planner_params);
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
    ros_path.header.stamp = now();
    ros_path.header.frame_id = latest_map_->header.frame_id;

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

    controller::ControllerParams controller_params;
    controller_params.max_linear_velocity = 0.8;
    controller_params.max_angular_velocity = 2.0;
    controller_params.max_angular_acceleration = 1.8;
    controller_params.approach_velocity_scaling_dist = 1.0;
    controller_params.proximity_distance = 0.3;
    controller_params.distance_tolerance = 0.1;

    controller_.setParameters(&local_costmap_, &collision_checker_,
                              &trig_table_, controller_params);

    auto start_pose = geometry::Pose3d(start_x_, start_y_, start_theta_);
    auto goal_pose = geometry::Pose3d(goal_x_, goal_y_, goal_theta_);
    auto [v, w] = controller_.computeCommand(
        start_pose, goal_pose, linear_velocity_, angular_velocity_, path_);

    RCLCPP_INFO(get_logger(), "Distance to goal: %lf",
                utils::distance(start_pose,
                                goal_pose));
    RCLCPP_INFO(get_logger(), "Publishing v: %lf and w: %lf", v, w);

    geometry::Pose2d lookahead_point = controller_.getLookaheadPoint();

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = latest_map_->header.frame_id;
    marker.header.stamp = get_clock()->now();

    marker.ns = "lookahead";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = lookahead_point.x;
    marker.pose.position.y = lookahead_point.y;
    marker.pose.position.z = 0.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0);

    lookahead_pose_pub_->publish(marker);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;

    cmd_vel_pub_->publish(cmd);
  }

private:
  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr costmap_timer_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      lookahead_pose_pub_;
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

  double start_x_, start_y_, start_theta_;
  double goal_x_, goal_y_, goal_theta_;

  bool have_start_ = false;
  bool have_goal_ = false;

  double linear_velocity_, angular_velocity_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation>());
  rclcpp::shutdown();
  return 0;
}
