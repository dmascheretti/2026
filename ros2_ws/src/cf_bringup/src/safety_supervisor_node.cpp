// ROS 2 wrapper around cf_bringup::SafetySupervisor. The ONLY node that
// talks to the drone.
//
// Subscribes:
//   ekf/odom      nav_msgs/Odometry        (freshness + geofence)
//   mpc/command   cf_msgs/AttitudeCommand
//   kill          std_msgs/Empty           kill switch
// Publishes at 100 Hz:
//   cmd_vel_legacy  geometry_msgs/Twist    to Crazyswarm2 (or the sim bridge)
//   safety/active   std_msgs/Bool          true while commands are forwarded
// Service:
//   safety/arm      std_srvs/Trigger       start forwarding MPC commands
//
// Before arming and after any stop, setpoints with zero thrust are sent:
// the motors stay off, and the firmware's thrust lock is released
// (it needs one zero-thrust setpoint before it accepts thrust).
// Freshness is judged by arrival time on this node's clock.
#include <memory>
#include <string>

#include <cf_msgs/msg/attitude_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cf_bringup/crazyflie_interface.hpp"
#include "cf_bringup/safety_supervisor.hpp"

class SafetySupervisorNode : public rclcpp::Node {
 public:
  SafetySupervisorNode() : Node("safety_supervisor") {
    const std::string config_file = declare_parameter<std::string>("safety_config_file");
    const double rate = declare_parameter<double>("rate", 100.0);
    supervisor_ = std::make_unique<cf_bringup::SafetySupervisor>(
        cf_bringup::load_safety_config(config_file));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_legacy", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("safety/active", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ekf/odom", 10, [this](const nav_msgs::msg::Odometry& msg) {
          position_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                      msg.pose.pose.position.z);
          estimate_stamp_ = now_seconds();
        });
    command_sub_ = create_subscription<cf_msgs::msg::AttitudeCommand>(
        "mpc/command", 10, [this](const cf_msgs::msg::AttitudeCommand& msg) {
          command_.roll = msg.roll;
          command_.pitch = msg.pitch;
          command_.yaw_rate = msg.yaw_rate;
          command_.thrust_cmd = msg.thrust_cmd;
          solver_ok_ = msg.solver_ok;
          command_stamp_ = now_seconds();
        });
    kill_sub_ = create_subscription<std_msgs::msg::Empty>(
        "kill", 10, [this](const std_msgs::msg::Empty&) { kill_ = true; });
    arm_srv_ = create_service<std_srvs::srv::Trigger>(
        "safety/arm", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          kill_ = false;
          supervisor_->reset();
          armed_ = true;
          stop_reported_ = false;
          response->success = true;
          response->message = "armed";
          RCLCPP_WARN(get_logger(), "ARMED: forwarding MPC commands");
        });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { on_timer(); });
  }

 private:
  double now_seconds() { return now().seconds(); }

  void on_timer() {
    cf_bringup::Command out_command;  // zero = motors off
    bool active = false;

    if (armed_) {
      cf_bringup::SupervisorInput in;
      in.now = now_seconds();
      in.position = position_;
      in.estimate_stamp = estimate_stamp_;
      in.command = command_;
      in.command_stamp = command_stamp_;
      in.solver_ok = solver_ok_;
      in.kill_switch = kill_;
      const cf_bringup::SupervisorOutput out = supervisor_->step(in);
      out_command = out.command;
      active = (out.mode == cf_bringup::Mode::kActive);
      if (!active && !stop_reported_) {
        RCLCPP_ERROR(get_logger(), "STOP: %s. Motors off. Call safety/arm to fly again.",
                     out.stop_reason.c_str());
        stop_reported_ = true;
      }
    }

    const cf_bringup::LegacyTwist legacy = cf_bringup::to_cmd_vel_legacy(out_command);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = legacy.linear_x;
    twist.linear.y = legacy.linear_y;
    twist.linear.z = legacy.linear_z;
    twist.angular.z = legacy.angular_z;
    cmd_pub_->publish(twist);

    std_msgs::msg::Bool active_msg;
    active_msg.data = active;
    active_pub_->publish(active_msg);
  }

  std::unique_ptr<cf_bringup::SafetySupervisor> supervisor_;
  bool armed_ = false;
  bool kill_ = false;
  bool stop_reported_ = false;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  double estimate_stamp_ = -1.0;
  cf_bringup::Command command_;
  double command_stamp_ = -1.0;
  bool solver_ok_ = true;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<cf_msgs::msg::AttitudeCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr kill_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
