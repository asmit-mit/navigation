#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include "voronoi/VoronoiImage.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

class Planner : public rclcpp::Node {
public:
  Planner() : Node("planner") {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&Planner::mapCallback, this, _1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/voronoi_costmap", qos);

    timer_ = this->create_wall_timer(200ms,
                                     std::bind(&Planner::timerCallback, this));
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = msg;
  }

  void timerCallback() {
    if (!latest_map_)
      return;
    voronoi_.setGrid(latest_map_);
    voronoi_.ComputeFT();

    nav_msgs::msg::OccupancyGrid costmap;

    costmap.header.stamp = this->now();
    costmap.header.frame_id = latest_map_->header.frame_id; 

    costmap.info = latest_map_->info;
    costmap.data.resize(costmap.info.width * costmap.info.height);

    for (unsigned int y = 0; y < costmap.info.height; y++) {
      for (unsigned int x = 0; x < costmap.info.width; x++) {
        int idx = y * costmap.info.width + x;
        costmap.data[idx] = voronoi_.isEdge(x, y) * 100;
      }
    }

    costmap_pub_->publish(costmap);

    RCLCPP_INFO(this->get_logger(), "Computed Voronoi");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  voronoi::VoronoiImage voronoi_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
