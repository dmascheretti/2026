// ROS 2 wrapper around cf_ekf::Ekf.
//
// Subscribes (Crazyswarm2 custom log topics, see cf_bringup/config):
//   imu   crazyflie_interfaces/LogDataGeneric  [acc.x..z (g), gyro.x..z (deg/s)]
//   flow  crazyflie_interfaces/LogDataGeneric  [motion.deltaX, motion.deltaY, range.zrange (mm)]
// Publishes:
//   ekf/odom  nav_msgs/Odometry. Pose in the world frame. The twist is the
//             WORLD-frame velocity, so child_frame_id is also the world frame.
// Service:
//   ekf/reset std_srvs/Trigger  reset to the initial position (before take-off)
//
// Time steps come from the on-board timestamps of the log packets, not from
// the arrival time, so radio jitter does not enter the prediction.
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cf_ekf/crazyflie_logs.hpp"
#include "cf_ekf/ekf.hpp"
#include "cf_model/params.hpp"

using crazyflie_interfaces::msg::LogDataGeneric;

class EkfNode : public rclcpp::Node {
 public:
  EkfNode() : Node("ekf") {
    const std::string params_file = declare_parameter<std::string>("params_file");
    const std::string ekf_config_file = declare_parameter<std::string>("ekf_config_file");
    const std::vector<double> p0 = declare_parameter<std::vector<double>>(
        "initial_position", std::vector<double>{0.0, 0.0, 0.0});
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    initial_position_ = Eigen::Vector3d(p0.at(0), p0.at(1), p0.at(2));

    params_ = cf_model::load_params(params_file);
    ekf_ = std::make_unique<cf_ekf::Ekf>(params_, cf_ekf::load_ekf_config(ekf_config_file));
    reset_filter();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("ekf/odom", 10);
    imu_sub_ = create_subscription<LogDataGeneric>(
        "imu", rclcpp::SensorDataQoS(), [this](const LogDataGeneric& msg) { on_imu(msg); });
    flow_sub_ = create_subscription<LogDataGeneric>(
        "flow", rclcpp::SensorDataQoS(), [this](const LogDataGeneric& msg) { on_flow(msg); });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "ekf/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          reset_filter();
          response->success = true;
          response->message = "EKF reset to initial position";
        });
  }

 private:
  void reset_filter() {
    cf_ekf::StateVector x0 = cf_ekf::StateVector::Zero();
    x0.segment<3>(cf_ekf::PX) = initial_position_;
    ekf_->reset(x0);
    last_imu_stamp_ms_ = -1;
    last_flow_stamp_ms_ = -1;
    RCLCPP_INFO(get_logger(), "EKF reset at (%.2f, %.2f, %.2f)", initial_position_.x(),
                initial_position_.y(), initial_position_.z());
  }

  void on_imu(const LogDataGeneric& msg) {
    const cf_ekf::ImuSample imu = cf_ekf::imu_from_log(msg.values, params_.gravity);
    last_gyro_ = imu.gyro;
    const int64_t stamp_ms = msg.timestamp;
    if (last_imu_stamp_ms_ >= 0) {
      const double dt = (stamp_ms - last_imu_stamp_ms_) / 1000.0;
      if (dt > 0.0 && dt < 0.1) {
        ekf_->predict(imu, dt);
      } else {
        RCLCPP_WARN(get_logger(), "IMU time step %.3f s ignored", dt);
      }
    }
    last_imu_stamp_ms_ = stamp_ms;
    publish();
  }

  void on_flow(const LogDataGeneric& msg) {
    const int64_t stamp_ms = msg.timestamp;
    ekf_->update_range(cf_ekf::range_from_log(msg.values));
    if (last_flow_stamp_ms_ >= 0) {
      cf_ekf::FlowSample flow;
      flow.dt = (stamp_ms - last_flow_stamp_ms_) / 1000.0;
      const Eigen::Vector2d pixels = cf_ekf::flow_pixels_from_log(msg.values);
      flow.dpixel_x = pixels.x();
      flow.dpixel_y = pixels.y();
      flow.gyro = last_gyro_;
      ekf_->update_flow(flow);
    }
    last_flow_stamp_ms_ = stamp_ms;
  }

  void publish() {
    const cf_ekf::StateVector& x = ekf_->state();
    const Eigen::Quaterniond q(cf_model::rotation_zyx(x(cf_ekf::ROLL), x(cf_ekf::PITCH), x(cf_ekf::YAW)));
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = frame_id_;  // twist is in the world frame
    odom.pose.pose.position.x = x(cf_ekf::PX);
    odom.pose.pose.position.y = x(cf_ekf::PY);
    odom.pose.pose.position.z = x(cf_ekf::PZ);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.twist.twist.linear.x = x(cf_ekf::VX);
    odom.twist.twist.linear.y = x(cf_ekf::VY);
    odom.twist.twist.linear.z = x(cf_ekf::VZ);
    // Diagonal variances: position + attitude in pose, velocity in twist.
    const cf_ekf::StateMatrix& P = ekf_->covariance();
    for (int i = 0; i < 3; ++i) {
      odom.pose.covariance[i * 6 + i] = P(cf_ekf::PX + i, cf_ekf::PX + i);
      odom.pose.covariance[(i + 3) * 6 + (i + 3)] = P(cf_ekf::ROLL + i, cf_ekf::ROLL + i);
      odom.twist.covariance[i * 6 + i] = P(cf_ekf::VX + i, cf_ekf::VX + i);
    }
    odom_pub_->publish(odom);
  }

  cf_model::Params params_;
  std::unique_ptr<cf_ekf::Ekf> ekf_;
  Eigen::Vector3d initial_position_;
  std::string frame_id_;
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
  int64_t last_imu_stamp_ms_ = -1;
  int64_t last_flow_stamp_ms_ = -1;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<LogDataGeneric>::SharedPtr imu_sub_;
  rclcpp::Subscription<LogDataGeneric>::SharedPtr flow_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
