#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

constexpr double PI = 3.14159265358979323846;

class TerminalGuard
{
public:
  TerminalGuard()
  {
    if (!isatty(STDIN_FILENO))
    {
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_) != 0)
    {
      return;
    }

    termios raw = original_;

    raw.c_lflag &=
      static_cast<tcflag_t>(~(ICANON | ECHO));

    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
    {
      active_ = true;
    }
  }

  ~TerminalGuard()
  {
    if (active_)
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard & operator=(const TerminalGuard &) = delete;

private:
  termios original_{};
  bool active_{false};
};

enum class AugerMode
{
  REVERSE = -1,
  NEUTRAL = 0,
  FORWARD = 1
};

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("science_2026_keyboard_teleop")
  {
    lift_command_magnitude_ =
      declare_parameter<double>(
      "lift_command_magnitude",
      1.0);

    auger_velocity_magnitude_ =
      declare_parameter<double>(
      "auger_velocity_magnitude",
      1.0);

    hold_timeout_ms_ =
      declare_parameter<int>(
      "lift_hold_timeout_ms",
      180);

    velocity_command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      "/science_velocity_controller/commands",
      10);

    tray_command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      "/beaker_position_controller/commands",
      10);

    publish_velocity_command();
    publish_tray_command();

    print_help();
    print_status();
  }

  void run()
  {
    if (!isatty(STDIN_FILENO))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Keyboard teleop requires a real terminal.");

      return;
    }

    TerminalGuard terminal_guard;
    rclcpp::WallRate loop_rate(50.0);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());

      char key = 0;

      if (read_key(key))
      {
        handle_key(key);
      }

      update_lift_hold();
      loop_rate.sleep();
    }

    stop_all_motion();
  }

private:
  bool read_key(char & key) const
  {
    fd_set read_set;

    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = select(
      STDIN_FILENO + 1,
      &read_set,
      nullptr,
      nullptr,
      &timeout);

    if (ready < 0)
    {
      if (errno != EINTR)
      {
        RCLCPP_WARN(
          get_logger(),
          "select() failed: %s",
          std::strerror(errno));
      }

      return false;
    }

    if (ready == 0)
    {
      return false;
    }

    const ssize_t bytes_read =
      ::read(STDIN_FILENO, &key, 1);

    return bytes_read == 1;
  }

  void handle_key(char key)
  {
    switch (key)
    {
      // ---------------------------------------------------------
      // Linear actuator manual control
      // ---------------------------------------------------------

      case 'w':
      case 'W':
        set_lift_hold(lift_command_magnitude_);
        break;

      case 's':
      case 'S':
        set_lift_hold(-lift_command_magnitude_);
        break;

      // ---------------------------------------------------------
      // Auger toggle state machine
      // ---------------------------------------------------------

      case 'q':
      case 'Q':
        handle_auger_forward_key();
        break;

      case 'a':
      case 'A':
        handle_auger_reverse_key();
        break;

      // ---------------------------------------------------------
      // Beaker tray cycle
      // ---------------------------------------------------------

      case 'e':
      case 'E':
        cycle_tray();
        break;

      // ---------------------------------------------------------
      // Emergency motion stop
      // ---------------------------------------------------------

      case ' ':
        stop_all_motion();
        break;

      // ---------------------------------------------------------
      // Help
      // ---------------------------------------------------------

      case 'h':
      case 'H':
        print_help();
        print_status();
        break;

      default:
        break;
    }
  }

  // =========================================================================
  // LINEAR ACTUATOR
  // =========================================================================

  void set_lift_hold(double command)
  {
    lift_velocity_command_ = command;

    lift_hold_deadline_ =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(hold_timeout_ms_);

    publish_velocity_command();
    print_status();
  }

  void update_lift_hold()
  {
    if (std::fabs(lift_velocity_command_) < 1e-9)
    {
      return;
    }

    if (
      std::chrono::steady_clock::now() >
      lift_hold_deadline_)
    {
      lift_velocity_command_ = 0.0;

      publish_velocity_command();
      print_status();
    }
  }

  // =========================================================================
  // AUGER STATE MACHINE
  // =========================================================================

  void handle_auger_forward_key()
  {
    if (auger_mode_ == AugerMode::REVERSE)
    {
      auger_mode_ = AugerMode::NEUTRAL;
    }
    else
    {
      auger_mode_ = AugerMode::FORWARD;
    }

    update_auger_command();
  }

  void handle_auger_reverse_key()
  {
    if (auger_mode_ == AugerMode::FORWARD)
    {
      auger_mode_ = AugerMode::NEUTRAL;
    }
    else
    {
      auger_mode_ = AugerMode::REVERSE;
    }

    update_auger_command();
  }

  void update_auger_command()
  {
    switch (auger_mode_)
    {
      case AugerMode::FORWARD:
        auger_velocity_command_ =
          auger_velocity_magnitude_;
        break;

      case AugerMode::REVERSE:
        auger_velocity_command_ =
          -auger_velocity_magnitude_;
        break;

      case AugerMode::NEUTRAL:
      default:
        auger_velocity_command_ = 0.0;
        break;
    }

    publish_velocity_command();
    print_status();
  }

  // =========================================================================
  // BEAKER TRAY
  // =========================================================================

  void cycle_tray()
  {
    tray_position_index_ =
      (tray_position_index_ + 1) % 3;

    switch (tray_position_index_)
    {
      case 0:
        tray_position_command_ = 0.0;
        break;

      case 1:
        tray_position_command_ = PI / 2.0;
        break;

      case 2:
        tray_position_command_ = PI;
        break;

      default:
        tray_position_index_ = 0;
        tray_position_command_ = 0.0;
        break;
    }

    publish_tray_command();
    print_status();
  }

  // =========================================================================
  // PUBLISHING
  // =========================================================================

  void publish_velocity_command()
  {
    std_msgs::msg::Float64MultiArray message;

    message.data = {
      lift_velocity_command_,
      auger_velocity_command_
    };

    velocity_command_pub_->publish(message);
  }

  void publish_tray_command()
  {
    std_msgs::msg::Float64MultiArray message;

    message.data = {
      tray_position_command_
    };

    tray_command_pub_->publish(message);
  }

  void stop_all_motion()
  {
    lift_velocity_command_ = 0.0;
    auger_velocity_command_ = 0.0;
    auger_mode_ = AugerMode::NEUTRAL;

    publish_velocity_command();

    RCLCPP_INFO(
      get_logger(),
      "Lift and auger stopped.");

    print_status();
  }

  // =========================================================================
  // TERMINAL OUTPUT
  // =========================================================================

  const char * auger_mode_string() const
  {
    switch (auger_mode_)
    {
      case AugerMode::FORWARD:
        return "FORWARD";

      case AugerMode::REVERSE:
        return "REVERSE";

      case AugerMode::NEUTRAL:
      default:
        return "NEUTRAL";
    }
  }

  const char * lift_mode_string() const
  {
    if (lift_velocity_command_ > 0.0)
    {
      return "EXTEND";
    }

    if (lift_velocity_command_ < 0.0)
    {
      return "RETRACT";
    }

    return "STOP";
  }

  void print_help() const
  {
    std::printf(
      "\n"
      "Science keyboard teleop\n"
      "\n"
      "  Hold W : Extend lift actuator\n"
      "  Hold S : Retract lift actuator\n"
      "\n"
      "  Q      : Neutral -> auger forward\n"
      "           Auger reverse -> neutral\n"
      "\n"
      "  A      : Neutral -> auger reverse\n"
      "           Auger forward -> neutral\n"
      "\n"
      "  E      : Cycle tray 0 -> 90 -> 180 -> 0 degrees\n"
      "\n"
      "  Space  : Stop lift and auger\n"
      "  H      : Show help\n"
      "  Ctrl-C : Exit safely\n"
      "\n");

    std::fflush(stdout);
  }

  void print_status() const
  {
    const double tray_degrees =
      tray_position_command_ * 180.0 / PI;

    std::printf(
      "\rLift: %-7s | Auger: %-7s | Tray: %3.0f degrees       ",
      lift_mode_string(),
      auger_mode_string(),
      tray_degrees);

    std::fflush(stdout);
  }

  // =========================================================================
  // ROS INTERFACES
  // =========================================================================

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
  velocity_command_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
  tray_command_pub_;

  // =========================================================================
  // COMMAND STATE
  // =========================================================================

  double lift_velocity_command_{0.0};
  double auger_velocity_command_{0.0};
  double tray_position_command_{0.0};

  AugerMode auger_mode_{AugerMode::NEUTRAL};

  int tray_position_index_{0};

  std::chrono::steady_clock::time_point
    lift_hold_deadline_{};

  // =========================================================================
  // PARAMETERS
  // =========================================================================

  double lift_command_magnitude_{1.0};
  double auger_velocity_magnitude_{1.0};

  int hold_timeout_ms_{180};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<KeyboardTeleop>();

  node->run();

  rclcpp::shutdown();
  return 0;
}
