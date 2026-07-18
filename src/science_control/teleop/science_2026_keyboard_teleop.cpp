#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

class TerminalGuard
{
public:
  TerminalGuard()
  {
    if (!isatty(STDIN_FILENO)) {
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_) != 0) {
      return;
    }

    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      active_ = true;
    }
  }

  ~TerminalGuard()
  {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard & operator=(const TerminalGuard &) = delete;

private:
  termios original_{};
  bool active_{false};
};

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("science_2026_keyboard_teleop")
  {
    publish_controller_commands_ = this->declare_parameter<bool>(
      "publish_controller_commands", true);
    require_joint_state_before_motion_ = this->declare_parameter<bool>(
      "require_joint_state_before_motion", false);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", 10);

    debug_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/science_2026/debug_mode", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KeyboardTeleop::joint_state_callback, this, std::placeholders::_1));

    print_help();
  }

  void spin()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_ERROR(get_logger(), "Keyboard teleop requires a real terminal (TTY).");
      return;
    }

    TerminalGuard terminal_guard;
    rclcpp::WallRate loop_rate(50.0);

    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());

      char key = 0;
      if (read_key(key)) {
        if (!handle_key(key)) {
          break;
        }
      }

      update_hold_commands();
      loop_rate.sleep();
    }

    target_positions_[1] = 0.0;
    publish_target();
  }

private:
  void print_help() const
  {
    std::printf(
      "\n"
      "Keyboard teleop for arm_2026\n"
      "Commands:\n"
      "  q/a : base yaw + / -\n"
      "  w/s : shoulder actuator + / - while held\n"
      "  e   : toggle elbow servo (0° -> 90° -> 180° -> 0°)\n"
      "  r/f : wrist twist + / -\n"
      "  t/g : wrist roll + / -\n"
      "  y/h : claw + / -\n"
      "  z   : decrease step size\n"
      "  x   : increase step size\n"
      "  p   : toggle debug mode\n"
      "  space : publish current targets\n"
      "  c   : set targets to measured positions\n"
      "  Ctrl-C : quit\n"
      "\n");
    std::printf(
      "Publish mode: /position_controller/commands=%s, require_joint_state=%s, debug=%s\n",
      publish_controller_commands_ ? "on" : "off",
      require_joint_state_before_motion_ ? "on" : "off",
      debug_mode_enabled_ ? "ON" : "OFF");
    print_status();
  }

  void print_status() const
  {
    // Show servo position in degrees
    double servo_deg = target_positions_[2] * 180.0 / M_PI;
    std::printf(
      "Target [rad] base=%.3f shoulder=%.3f elbow=%.3f (%.0f°) wrist_roll=%.3f wrist_twist=%.3f claw=%.3f | step=%.3f rad (%.1f deg)\n",
      target_positions_[0],
      target_positions_[1],
      target_positions_[2],
      servo_deg,
      target_positions_[3],
      target_positions_[4],
      target_positions_[5],
      step_size_rad_,
      step_size_rad_ * 180.0 / M_PI);
    std::fflush(stdout);
  }

  void print_debug_banner() const
  {
    std::printf("\n");
    std::printf("########################################\n");
    std::printf("#                                      #\n");
    std::printf("#          DEBUG MODE %s           #\n", debug_mode_enabled_ ? "ON " : "OFF");
    std::printf("#                                      #\n");
    std::printf("########################################\n");
    std::printf("\n");
    std::fflush(stdout);
  }

  bool read_key(char & key) const
  {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready < 0) {
      if (errno != EINTR) {
        RCLCPP_WARN(
          get_logger(),
          "select() failed while reading keyboard input: %s",
          std::strerror(errno));
      }
      return false;
    }

    if (ready == 0) {
      return false;
    }

    const ssize_t bytes_read = ::read(STDIN_FILENO, &key, 1);
    return bytes_read == 1;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const std::array<std::string, 6> joint_names{
      "base_yaw",
      "shoulder_extension",
      "elbow_extension",
      "wrist_roll",
      "wrist_twist",
      "claw"};

    bool any_joint_updated = false;
    for (std::size_t target_index = 0; target_index < joint_names.size(); ++target_index) {
      for (std::size_t msg_index = 0; msg_index < msg->name.size(); ++msg_index) {
        if (msg->name[msg_index] == joint_names[target_index] &&
            msg_index < msg->position.size()) {
          measured_positions_[target_index] = msg->position[msg_index];
          any_joint_updated = true;
          break;
        }
      }
    }

    if (!have_joint_state_ && any_joint_updated) {
      target_positions_ = measured_positions_;
      have_joint_state_ = true;
      print_status();
    }
  }

  bool handle_key(char key)
  {
    switch (key) {
      case 'q':  // Base yaw +
        nudge_joint(0, step_size_rad_);
        return true;
      case 'a':  // Base yaw -
        nudge_joint(0, -step_size_rad_);
        return true;
      case 'w':  // Shoulder extend (hold)
        set_hold_joint(1, 1.0);
        return true;
      case 's':  // Shoulder retract (hold)
        set_hold_joint(1, -1.0);
        return true;
      case 'e':  // Toggle elbow servo (0 -> 90 -> 180 -> 0)
        toggle_servo();
        return true;
      case 'r':  // Wrist twist +
        nudge_joint(4, step_size_rad_);
        return true;
      case 'f':  // Wrist twist -
        nudge_joint(4, -step_size_rad_);
        return true;
      case 't':  // Wrist roll +
        nudge_joint(3, step_size_rad_);
        return true;
      case 'g':  // Wrist roll -
        nudge_joint(3, -step_size_rad_);
        return true;
      case 'y':  // Claw +
        nudge_joint(5, step_size_rad_);
        return true;
      case 'h':  // Claw -
        nudge_joint(5, -step_size_rad_);
        return true;
      case 'z':  // Decrease step size
        step_size_rad_ = std::max(min_step_size_rad_, step_size_rad_ / 2.0);
        print_status();
        return true;
      case 'x':  // Increase step size
        step_size_rad_ = std::min(max_step_size_rad_, step_size_rad_ * 2.0);
        print_status();
        return true;
      case 'p':  // Toggle debug mode
        toggle_debug_mode();
        return true;
      case ' ':  // Publish current targets
        publish_target();
        return true;
      case 'c':  // Set targets to measured positions
        target_positions_ = measured_positions_;
        publish_target();
        return true;
      default:
        return true;
    }
  }

  void nudge_joint(std::size_t joint_index, double delta)
  {
    if (!have_joint_state_ && require_joint_state_before_motion_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /joint_states before accepting teleop commands.");
      return;
    }

    if (debug_mode_enabled_) {
      target_positions_[joint_index] += delta;
    } else {
      target_positions_[joint_index] = std::clamp(
        target_positions_[joint_index] + delta,
        joint_mins_[joint_index],
        joint_maxs_[joint_index]);
    }

    publish_target();
  }

  // NEW: Toggle servo between 0°, 90°, 180°
  void toggle_servo()
  {
    if (!have_joint_state_ && require_joint_state_before_motion_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /joint_states before accepting teleop commands.");
      return;
    }

    // Get current angle in degrees
    double current_deg = target_positions_[2] * 180.0 / M_PI;
    
    // Round to nearest 90 degrees (0, 90, or 180)
    int current_step = static_cast<int>(std::round(current_deg / 90.0));
    current_step = std::clamp(current_step, 0, 2);  // 0, 1, or 2
    
    // Move to next step (0 -> 90 -> 180 -> 0)
    int next_step = (current_step + 1) % 3;
    double next_deg = next_step * 90.0;
    double next_rad = next_deg * M_PI / 180.0;
    
    target_positions_[2] = next_rad;
    publish_target();
    
    // Print status with degrees
    std::printf("Servo toggled to: %.0f°\n", next_deg);
  }

  void set_hold_joint(std::size_t joint_index, double value)
  {
    if (!have_joint_state_ && require_joint_state_before_motion_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /joint_states before accepting teleop commands.");
      return;
    }

    target_positions_[joint_index] = value;
    hold_deadline_[joint_index] = std::chrono::steady_clock::now() + hold_timeout_;
    publish_target();
  }

  void update_hold_commands()
  {
    const auto now = std::chrono::steady_clock::now();
    bool changed = false;

    for (std::size_t joint_index : {std::size_t(1)}) {  // Only shoulder
      if (std::fabs(target_positions_[joint_index]) > 1e-9 &&
          now > hold_deadline_[joint_index]) {
        target_positions_[joint_index] = 0.0;
        changed = true;
      }
    }

    if (changed) {
      publish_target();
    }
  }

  void toggle_debug_mode()
  {
    debug_mode_enabled_ = !debug_mode_enabled_;

    std_msgs::msg::Bool msg;
    msg.data = debug_mode_enabled_;
    debug_pub_->publish(msg);

    print_debug_banner();
    print_status();
  }

  void publish_target()
  {
    if (publish_controller_commands_ && command_pub_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(target_positions_.begin(), target_positions_.end());
      command_pub_->publish(msg);
    }

    print_status();
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::array<double, 6> measured_positions_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> target_positions_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<std::chrono::steady_clock::time_point, 6> hold_deadline_{};

  const std::array<double, 6> joint_mins_{{-3.14, -1.57, 0.0, -1.57, -1.57, -1.57}};
  const std::array<double, 6> joint_maxs_{{ 3.14,  1.57, 3.14,  1.57,  1.57,  1.57}};

  double step_size_rad_{10.0 * M_PI / 180.0};
  const double min_step_size_rad_{1.0 * M_PI / 180.0};
  const double max_step_size_rad_{45.0 * M_PI / 180.0};

  const std::chrono::milliseconds hold_timeout_{180};

  bool have_joint_state_{false};
  bool publish_controller_commands_{true};
  bool require_joint_state_before_motion_{false};
  bool debug_mode_enabled_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}