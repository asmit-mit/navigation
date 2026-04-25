#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry/collison_checker.h"
#include "geometry/pose.h"
#include "grid/global_costmap.h"
#include "grid/local_costmap.h"
#include "planner/hybrid_astar.h"
#include "planner/motion_model.h"
#include "planner/optimizer.h"
#include "utils/EDT.h"
#include "utils/trig_utils.h"

using namespace std::chrono_literals;

class TestPlanner : public rclcpp::Node {
public:
  TestPlanner() : Node("test_planner") {
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos, std::bind(&TestPlanner::mapCallback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&TestPlanner::startCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&TestPlanner::goalCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/nav/planner/path", 10);

    RCLCPP_INFO(this->get_logger(), "Test Planner Node Started");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::Pose start_;
  geometry_msgs::msg::Pose goal_;

  bool has_map_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;

  planner::HybridAStar planner_;
  planner::MotionModel motion_model_;
  planner::Optimizer optimizer_;
  geometry::CollisionChecker collision_checker_;
  grid::GlobalCostmap global_costmap_;
  grid::LocalCostmap local_costmap_;
  utils::TrigTable trig_table_ = utils::TrigTable(10000);
  utils::EDT edt_;

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = msg;

    RCLCPP_INFO(this->get_logger(),
                "Map received: %d x %d, resolution %.3f",
                msg->info.width,
                msg->info.height,
                msg->info.resolution);
    has_map_ = true;
    tryPlan();
  }

  void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    start_ = msg->pose.pose;

    RCLCPP_INFO(
        this->get_logger(), "Start received: (%.2f, %.2f)", start_.position.x, start_.position.y);

    has_start_ = true;
    tryPlan();
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_ = msg->pose;

    RCLCPP_INFO(
        this->get_logger(), "Goal received: (%.2f, %.2f)", goal_.position.x, goal_.position.y);

    has_goal_ = true;
    tryPlan();
  }

  void tryPlan() {
    if (!(has_map_ && has_start_ && has_goal_))
      return;

    RCLCPP_INFO(this->get_logger(), "Planning triggered");

    double start_x = start_.position.x;
    double start_y = start_.position.y;
    double start_theta = tf2::getYaw(start_.orientation);

    double goal_x = goal_.position.x;
    double goal_y = goal_.position.y;
    double goal_theta = tf2::getYaw(goal_.orientation);

    edt_.computeDT(map_);
    global_costmap_.setParameters(map_, &edt_, 1.5, 0.4, 3.0);
    collision_checker_.setParameters(&global_costmap_, nullptr, 0.01);

    planner::HybridAstarParams params;

    optimizer_.setCostmap(&global_costmap_);
    optimizer_.setIterations(1000);
    optimizer_.setWeights(0.3, 0.2);

    motion_model_.setTrigTable(&trig_table_);
    motion_model_.setMotionModel(planner::MotionModelType::DUBINS);
    motion_model_.setDistanceResolution(1.41421356 * global_costmap_.getResolution());
    motion_model_.setMinTurningRadius(params.max_linear_velocity / params.max_angular_velocity);
    motion_model_.setTolerance(params.distance_tolerance, params.angular_tolerance);

    planner_.setParameters(
        &global_costmap_, &optimizer_, &motion_model_, &collision_checker_, &trig_table_, params);

    planner_.setStart(start_x, start_y, start_theta);
    planner_.setGoal(goal_x, goal_y, goal_theta);

    auto start_time = std::chrono::high_resolution_clock::now();

    auto path = planner_.getPlan();

    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    RCLCPP_INFO(this->get_logger(), "Planning took %.3f ms", duration_ms);

    publishPath(path);
  }

  void publishPath(const std::vector<geometry::Pose3d> &path_vec) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (const auto &p : path_vec) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;

      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.position.z = 0.0;

      double half_theta = p.theta * 0.5;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = sin(half_theta);
      pose.pose.orientation.w = cos(half_theta);

      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path_vec.size());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPlanner>());
  rclcpp::shutdown();
  return 0;
}
