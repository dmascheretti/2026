// Simulated Crazyflie behind a fake Crazyswarm2 server: lets the complete
// ROS pipeline (EKF, MPC, supervisor, mission) run unchanged against the
// simulator instead of the radio.
//
// Plays the role of Crazyswarm2 + firmware:
//   cmd_vel_legacy (Twist) -> firmware setpoint, with the same unit/sign
//   chain (cf_bringup::from_cmd_vel_legacy), the firmware thrust lock and
//   the firmware setpoint watchdog (level after 0.5 s, motors off after 2 s).
// Publishes the same log topics as Crazyswarm2 custom logging:
//   imu   LogDataGeneric [acc.x..z (g), gyro.x..z (deg/s)]        100 Hz
//   flow  LogDataGeneric [motion.deltaX, motion.deltaY, range.zrange (mm)] 100 Hz
// plus ground_truth (nav_msgs/Odometry, world-frame twist) for analysis.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cf_bringup/crazyflie_interface.hpp"
#include "cf_model/params.hpp"
#include "plant.hpp"

using crazyflie_interfaces::msg::LogDataGeneric;

class SimBridgeNode : public rclcpp::Node {
 public:
  SimBridgeNode() : Node("sim_bridge") {
    const std::string params_file = declare_parameter<std::string>("params_file");
    const std::string sim_config_file = declare_parameter<std::string>("sim_config_file");
    const std::vector<double> p0 = declare_parameter<std::vector<double>>(
        "initial_position", std::vector<double>{0.0, 0.0, 0.0});
    quantize_flow_ = declare_parameter<bool>("quantize_flow", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    params_ = cf_model::load_params(params_file);
    config_ = sim::load_sim_config(sim_config_file);
    quad_ = std::make_unique<sim::Quadrotor>(params_, config_);
    quad_->set_position(Eigen::Vector3d(p0.at(0), p0.at(1), p0.at(2)));

    imu_pub_ = create_publisher<LogDataGeneric>("imu", rclcpp::SensorDataQoS());
    flow_pub_ = create_publisher<LogDataGeneric>("flow", rclcpp::SensorDataQoS());
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_legacy", 10, [this](const geometry_msgs::msg::Twist& msg) { on_setpoint(msg); });

    // Physics runs in 2 ms ticks of 2 x 1 ms steps; logs every 5th tick (100 Hz).
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() { on_tick(); });
  }

 private:
  void on_setpoint(const geometry_msgs::msg::Twist& msg) {
    cf_bringup::LegacyTwist twist;
    twist.linear_x = msg.linear.x;
    twist.linear_y = msg.linear.y;
    twist.linear_z = msg.linear.z;
    twist.angular_z = msg.angular.z;
    const cf_bringup::Command command = cf_bringup::from_cmd_vel_legacy(twist);

    // Firmware thrust lock: thrust is ignored until one zero-thrust setpoint.
    if (command.thrust_cmd == 0.0) {
      thrust_unlocked_ = true;
    }
    setpoint_.roll = command.roll;
    setpoint_.pitch = command.pitch;
    setpoint_.yaw_rate = command.yaw_rate;
    setpoint_.thrust_cmd = thrust_unlocked_ ? command.thrust_cmd : 0.0;
    last_setpoint_time_ms_ = sim_time_ms_;
  }

  void on_tick() {
    // Firmware setpoint watchdog.
    sim::Setpoint applied = setpoint_;
    const int64_t age_ms = sim_time_ms_ - last_setpoint_time_ms_;
    if (last_setpoint_time_ms_ < 0 || age_ms > 2000) {
      applied = sim::Setpoint{};  // motors off
    } else if (age_ms > 500) {
      applied.roll = 0.0;
      applied.pitch = 0.0;
      applied.yaw_rate = 0.0;
    }
    quad_->set_setpoint(applied);

    for (int i = 0; i < 2; ++i) {
      quad_->step(0.001);
      ++sim_time_ms_;
    }
    ++tick_;
    if (tick_ % 5 == 0) {
      publish_logs();
    }
  }

  void publish_logs() {
    const uint32_t stamp = static_cast<uint32_t>(sim_time_ms_);

    const Eigen::Vector3d accel = quad_->measure_accel() / params_.gravity;  // g
    const Eigen::Vector3d gyro = quad_->measure_gyro() * 180.0 / M_PI;       // deg/s
    LogDataGeneric imu;
    imu.header.stamp = now();
    imu.timestamp = stamp;
    imu.values = {static_cast<float>(accel.x()), static_cast<float>(accel.y()),
                  static_cast<float>(accel.z()), static_cast<float>(gyro.x()),
                  static_cast<float>(gyro.y()), static_cast<float>(gyro.z())};
    imu_pub_->publish(imu);

    // Flow: EKF convention -> raw sensor axes (inverse of the firmware's
    // dpixelx = -deltaY, dpixely = -deltaX), integer counts like the sensor.
    const Eigen::Vector2d pixels = quad_->measure_flow(0.01);
    double delta_x = -pixels.y();
    double delta_y = -pixels.x();
    if (quantize_flow_) {
      delta_x = std::round(delta_x);
      delta_y = std::round(delta_y);
    }
    const double range_mm = std::max(0.0, std::round(quad_->measure_range() * 1000.0));
    LogDataGeneric flow;
    flow.header.stamp = imu.header.stamp;
    flow.timestamp = stamp;
    flow.values = {static_cast<float>(delta_x), static_cast<float>(delta_y),
                   static_cast<float>(range_mm)};
    flow_pub_->publish(flow);

    const sim::TrueState& s = quad_->state();
    nav_msgs::msg::Odometry truth;
    truth.header.stamp = imu.header.stamp;
    truth.header.frame_id = frame_id_;
    truth.child_frame_id = frame_id_;  // twist in the world frame
    truth.pose.pose.position.x = s.position.x();
    truth.pose.pose.position.y = s.position.y();
    truth.pose.pose.position.z = s.position.z();
    truth.pose.pose.orientation.w = s.attitude.w();
    truth.pose.pose.orientation.x = s.attitude.x();
    truth.pose.pose.orientation.y = s.attitude.y();
    truth.pose.pose.orientation.z = s.attitude.z();
    truth.twist.twist.linear.x = s.velocity.x();
    truth.twist.twist.linear.y = s.velocity.y();
    truth.twist.twist.linear.z = s.velocity.z();
    truth_pub_->publish(truth);
  }

  cf_model::Params params_;
  sim::SimConfig config_;
  std::unique_ptr<sim::Quadrotor> quad_;
  bool quantize_flow_ = true;
  std::string frame_id_;
  sim::Setpoint setpoint_;
  bool thrust_unlocked_ = false;
  int64_t sim_time_ms_ = 0;
  int64_t last_setpoint_time_ms_ = -1;
  int64_t tick_ = 0;

  rclcpp::Publisher<LogDataGeneric>::SharedPtr imu_pub_;
  rclcpp::Publisher<LogDataGeneric>::SharedPtr flow_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
