// Keyboard kill switch. Run it in its own terminal:
//   ros2 run cf_bringup kill_switch_node --ros-args -r __ns:=/cf231
// Any key (SPACE is the intended one) publishes std_msgs/Empty on "kill".
// Hardware fallback if this node or ROS fails: stop the supervisor (Ctrl-C)
// or unplug the Crazyradio; with no setpoints the firmware levels the drone
// after 0.5 s and cuts the motors after 2 s.
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace {

// Puts the terminal in raw mode for the lifetime of the object, so a single
// key press is read without waiting for Enter.
class RawTerminal {
 public:
  RawTerminal() {
    tcgetattr(STDIN_FILENO, &original_);
    termios raw = original_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;   // read() returns immediately
    raw.c_cc[VTIME] = 1;  // ... or after 0.1 s
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~RawTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &original_); }

 private:
  termios original_{};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kill_switch");
  auto pub = node->create_publisher<std_msgs::msg::Empty>("kill", 10);

  if (!isatty(STDIN_FILENO)) {
    RCLCPP_FATAL(node->get_logger(), "kill_switch_node needs a terminal (run it with ros2 run)");
    return 1;
  }
  RawTerminal raw_terminal;
  RCLCPP_WARN(node->get_logger(), "Press SPACE (any key) to KILL the motors.");

  while (rclcpp::ok()) {
    char key = 0;
    if (read(STDIN_FILENO, &key, 1) == 1) {
      // Publish a few times in case one message is lost.
      for (int i = 0; i < 5; ++i) {
        pub->publish(std_msgs::msg::Empty());
      }
      RCLCPP_ERROR(node->get_logger(), "KILL sent");
    }
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
