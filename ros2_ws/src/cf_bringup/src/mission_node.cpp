// Plays a scenario file (same format as the simulator scenarios) as a
// position reference. The clock starts when the supervisor becomes active
// (after safety/arm), so take-off begins only once armed.
//
// Subscribes: safety/active (std_msgs/Bool)
// Publishes:  reference (geometry_msgs/PoseStamped) at 20 Hz
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cf_bringup/scenario.hpp"

class MissionNode : public rclcpp::Node {
 public:
  MissionNode() : Node("mission") {
    const std::string scenario_file = declare_parameter<std::string>("scenario_file");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    scenario_ = cf_bringup::load_scenario(scenario_file);

    reference_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("reference", 10);
    active_sub_ = create_subscription<std_msgs::msg::Bool>(
        "safety/active", 10, [this](const std_msgs::msg::Bool& msg) {
          if (msg.data && !start_time_) {
            start_time_ = now().seconds();
            RCLCPP_INFO(get_logger(), "mission started");
          }
          if (!msg.data) {
            start_time_.reset();  // restart from the first waypoint after re-arming
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { on_timer(); });
  }

 private:
  void on_timer() {
    const double t = start_time_ ? now().seconds() - *start_time_ : 0.0;
    const cf_bringup::Waypoint& wp = cf_bringup::waypoint_at(scenario_, t);
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = wp.position.x();
    msg.pose.position.y = wp.position.y();
    msg.pose.position.z = wp.position.z();
    msg.pose.orientation.w = std::cos(wp.yaw / 2.0);
    msg.pose.orientation.z = std::sin(wp.yaw / 2.0);
    reference_pub_->publish(msg);
  }

  cf_bringup::Scenario scenario_;
  std::string frame_id_;
  std::optional<double> start_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
