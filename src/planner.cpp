#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planner/hybrid_astar.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

class Planner : public rclcpp::Node {
public:
  Planner() : Node("planner") {

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos, std::bind(&Planner::mapCallback, this, _1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&Planner::goalCallback, this, _1));

    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/hybrid_astar_path", 10);

    timer_ = this->create_wall_timer(500ms,
                                     std::bind(&Planner::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Planner node started");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = msg;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    if (!have_start_) {
      start_pose_ = msg;
      have_start_ = true;
      RCLCPP_INFO(this->get_logger(), "Start pose received");
      return;
    }

    if (!have_goal_) {
      goal_pose_ = msg;
      have_goal_ = true;
      RCLCPP_INFO(this->get_logger(), "Goal pose received");
      return;
    }

    start_pose_ = goal_pose_;
    goal_pose_ = msg;

    RCLCPP_INFO(this->get_logger(), "Updated start and goal");
  }

  void timerCallback() {

    if (!latest_map_ || !have_start_ || !have_goal_)
      return;

    double start_x = start_pose_->pose.position.x;
    double start_y = start_pose_->pose.position.y;

    tf2::Quaternion q_start;
    tf2::fromMsg(start_pose_->pose.orientation, q_start);
    double roll, pitch, start_theta;
    tf2::Matrix3x3(q_start).getRPY(roll, pitch, start_theta);

    double goal_x = goal_pose_->pose.position.x;
    double goal_y = goal_pose_->pose.position.y;

    tf2::Quaternion q_goal;
    tf2::fromMsg(goal_pose_->pose.orientation, q_goal);
    double goal_theta;
    tf2::Matrix3x3(q_goal).getRPY(roll, pitch, goal_theta);

    planner::HybridAStar planner(latest_map_);
    planner.setVelocities(0.2, 0.5);
    planner.setTolerance(0.5, 0.2);
    planner.setStart(start_x, start_y, start_theta);
    planner.setGoal(goal_x, goal_y, goal_theta);
    planner.setAngularResolution(5);

    std::vector<planner::Pose> path = planner.getPlan();

    if (path.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path found");
      return;
    }

    nav_msgs::msg::Path ros_path;
    ros_path.header.stamp = this->now();
    ros_path.header.frame_id = "odom";

    for (const auto &pose : path) {

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = ros_path.header;

      pose_stamped.pose.position.x = pose.x;
      pose_stamped.pose.position.y = pose.y;
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, pose.theta);
      pose_stamped.pose.orientation = tf2::toMsg(q);

      ros_path.poses.push_back(pose_stamped);
    }

    path_pub_->publish(ros_path);

    RCLCPP_INFO(this->get_logger(), "Path published (%ld poses)",
                ros_path.poses.size());
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  geometry_msgs::msg::PoseStamped::SharedPtr start_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_;

  bool have_start_ = false;
  bool have_goal_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
