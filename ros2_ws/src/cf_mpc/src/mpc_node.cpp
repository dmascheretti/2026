// ROS 2 wrapper around cf_mpc::MpcController.
//
// Subscribes:
//   ekf/odom   nav_msgs/Odometry          state estimate (twist = world-frame velocity)
//   reference  geometry_msgs/PoseStamped  position + yaw to hold
//   safety/active std_msgs/Bool           disturbance estimate reset on arming
// Publishes, at the MPC sample rate (50 Hz):
//   mpc/command  cf_msgs/AttitudeCommand  goes to the safety supervisor,
//                                         never directly to the drone
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <cf_msgs/msg/attitude_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cf_model/params.hpp"
#include "cf_mpc/mpc_controller.hpp"

namespace {

Eigen::Vector3d euler_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
  const Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  return cf_model::euler_zyx(quat.normalized().toRotationMatrix());
}

}  // namespace

class MpcNode : public rclcpp::Node {
 public:
  MpcNode() : Node("mpc") {
    const std::string params_file = declare_parameter<std::string>("params_file");
    params_ = cf_model::load_params(params_file);
    controller_ = std::make_unique<cf_mpc::MpcController>(params_);

    command_pub_ = create_publisher<cf_msgs::msg::AttitudeCommand>("mpc/command", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ekf/odom", 10, [this](const nav_msgs::msg::Odometry& msg) { on_odom(msg); });
    reference_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "reference", 10, [this](const geometry_msgs::msg::PoseStamped& msg) { on_reference(msg); });
    active_sub_ = create_subscription<std_msgs::msg::Bool>(
        "safety/active", 10, [this](const std_msgs::msg::Bool& msg) {
          if (msg.data && !was_active_) {
            controller_->reset_disturbance();  // fresh estimate for every flight
          }
          was_active_ = msg.data;
        });

    const auto period = std::chrono::duration<double>(controller_->sample_time());
    timer_ = create_wall_timer(period, [this]() { on_timer(); });
    RCLCPP_INFO(get_logger(), "MPC running at %.0f Hz, horizon %d steps",
                1.0 / controller_->sample_time(), controller_->horizon_steps());
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry& msg) {
    cf_mpc::VehicleState state;
    state.position = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                     msg.pose.pose.position.z);
    state.velocity = Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                                     msg.twist.twist.linear.z);
    const Eigen::Vector3d euler = euler_from_quaternion(msg.pose.pose.orientation);
    state.roll = euler.x();
    state.pitch = euler.y();
    state.yaw = euler.z();
    state_ = state;
  }

  void on_reference(const geometry_msgs::msg::PoseStamped& msg) {
    const Eigen::Vector3d position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    const double yaw = euler_from_quaternion(msg.pose.orientation).z();
    reference_ = cf_mpc::make_hold_reference(position, yaw, controller_->horizon_steps());
  }

  void on_timer() {
    if (!state_ || !reference_) {
      return;  // nothing to control yet; the supervisor's watchdog keeps motors off
    }
    const cf_mpc::MpcOutput out = controller_->compute(*state_, *reference_);
    if (!out.ok) {
      RCLCPP_WARN(get_logger(), "MPC solver failed (acados status %d)", out.solver_status);
    }
    cf_msgs::msg::AttitudeCommand cmd;
    cmd.header.stamp = now();
    cmd.roll = out.command.roll;
    cmd.pitch = out.command.pitch;
    cmd.yaw_rate = out.command.yaw_rate;
    cmd.thrust_cmd = out.command.thrust_cmd;
    cmd.solver_ok = out.ok;
    cmd.solve_time = out.solve_time;
    cmd.disturbance = out.disturbance;
    command_pub_->publish(cmd);
  }

  cf_model::Params params_;
  std::unique_ptr<cf_mpc::MpcController> controller_;
  std::optional<cf_mpc::VehicleState> state_;
  std::optional<cf_mpc::Reference> reference_;
  bool was_active_ = false;

  rclcpp::Publisher<cf_msgs::msg::AttitudeCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcNode>());
  rclcpp::shutdown();
  return 0;
}
