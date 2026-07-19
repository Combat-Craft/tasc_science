#include "science_control/science_2026_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace science_2026
{

// ============================================================================
// DESTRUCTOR
// ============================================================================
Science2026System::~Science2026System()
{
  cleanup_phidgets();
  close_esp32_serial();
}

// ============================================================================
// INITIALIZATION
// ============================================================================
hardware_interface::CallbackReturn Science2026System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  if (joint_names_.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "Expected %zu joints, got %zu",
      NUM_JOINTS,
      joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint_names_[BASE_IDX] != "base_yaw" ||
      joint_names_[SHOULDER_IDX] != "shoulder_extension" ||
      joint_names_[ELBOW_IDX] != "elbow_extension" ||
      joint_names_[WRIST_ROLL_IDX] != "wrist_roll" ||
      joint_names_[WRIST_TWIST_IDX] != "wrist_twist" ||
      joint_names_[CLAW_IDX] != "claw")
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "Joint order mismatch. Expected [base_yaw, shoulder_extension, elbow_extension, wrist_roll, wrist_twist, claw].");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(NUM_JOINTS, 0.0);
  hw_states_.assign(NUM_JOINTS, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// EXPORT STATE INTERFACES
// ============================================================================
std::vector<hardware_interface::StateInterface>
Science2026System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(NUM_JOINTS);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }

  return state_interfaces;
}

// ============================================================================
// EXPORT COMMAND INTERFACES
// ============================================================================
std::vector<hardware_interface::CommandInterface>
Science2026System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(NUM_JOINTS);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
double Science2026System::clampf(double x, double lo, double hi) const
{
  return std::clamp(x, lo, hi);
}

bool Science2026System::phidget_ok(PhidgetReturnCode code, const char * context) const
{
  if (code == EPHIDGET_OK)
  {
    return true;
  }

  const char * error_description = "Unknown error";
  Phidget_getErrorDescription(code, &error_description);

  RCLCPP_ERROR(
    rclcpp::get_logger("Science2026System"),
    "Phidget error in %s: %s",
    context,
    error_description);

  return false;
}

// ============================================================================
// REAL HARDWARE: ESP32 (Shoulder Motor + Elbow Servo)
// Opens serial connection
// ============================================================================
bool Science2026System::open_esp32_serial()
{
  close_esp32_serial();

  const int max_attempts = 3;
  int attempt = 0;
  
  while (attempt < max_attempts)
  {
    attempt++;
    
    esp32_serial_fd_ = ::open(
      esp32_serial_device_.c_str(),
      O_RDWR | O_NOCTTY | O_SYNC);

    if (esp32_serial_fd_ >= 0)
    {
      break;
    }

    if (attempt < max_attempts)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  if (esp32_serial_fd_ < 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Science2026System"),
      "ESP32 not found on %s after %d attempts. Running without it.",
      esp32_serial_device_.c_str(),
      max_attempts);
    return false;
  }

  termios tty{};
  if (tcgetattr(esp32_serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "tcgetattr failed on %s",
      esp32_serial_device_.c_str());
    close_esp32_serial();
    return false;
  }

  cfmakeraw(&tty);

  speed_t baud = B115200;
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(esp32_serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "tcsetattr failed on %s",
      esp32_serial_device_.c_str());
    close_esp32_serial();
    return false;
  }

  tcflush(esp32_serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(
    rclcpp::get_logger("Science2026System"),
    "ESP32 connected on %s (Shoulder Motor + Elbow Servo)",
    esp32_serial_device_.c_str());

  return true;
}

// ============================================================================
// REAL HARDWARE: ESP32
// Closes serial connection
// ============================================================================
void Science2026System::close_esp32_serial()
{
  if (esp32_serial_fd_ >= 0)
  {
    ::close(esp32_serial_fd_);
    esp32_serial_fd_ = -1;
  }
}

// ============================================================================
// REAL HARDWARE: Linear Actuator (shoulder)
// Converts position command to byte command
// ============================================================================
uint8_t Science2026System::command_to_byte(double value) const
{
  if (value > actuator_command_deadband_)
  {
    return ACT_EXTEND;
  }
  if (value < -actuator_command_deadband_)
  {
    return ACT_RETRACT;
  }
  return ACT_STOP;
}

// ============================================================================
// REAL HARDWARE: Servo (elbow)
// Convert angle (rad) to servo byte (0-180)
// ============================================================================
double Science2026System::angle_to_servo_byte(double angle_rad) const
{
  const double joint_angle_deg =
    angle_rad * 180.0 / M_PI;

  return clampf(
    joint_angle_deg + 90.0,
    0.0,
    180.0);
}

// ============================================================================
// REAL HARDWARE: ESP32
// Sends combined packet: [0xAA] [Shoulder_CMD] [Servo_Angle] [Checksum]
// ============================================================================
bool Science2026System::send_esp32_packet(uint8_t shoulder_cmd, uint8_t servo_angle)
{
  if (esp32_serial_fd_ < 0)
  {
    return false;
  }

  const uint8_t header = 0xAA;
  const uint8_t checksum = shoulder_cmd ^ servo_angle;
  const uint8_t packet[4] = {header, shoulder_cmd, servo_angle, checksum};

  const ssize_t written = ::write(esp32_serial_fd_, packet, sizeof(packet));

  if (written != static_cast<ssize_t>(sizeof(packet)))
  {
    return false;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("Science2026System"),
    "Sent ESP32 packet: 0x%02X 0x%02X 0x%02X 0x%02X (shoulder=%d, servo=%d°)",
    packet[0], packet[1], packet[2], packet[3],
    shoulder_cmd, servo_angle);

  return true;
}

// ============================================================================
// REAL HARDWARE: Phidget Stepper Motor (base_yaw)
// Cleans up stepper motor connection
// ============================================================================
void Science2026System::cleanup_phidgets()
{
  if (stepper_)
  {
    phidget_ok(PhidgetStepper_setEngaged(stepper_, 0), "setEngaged(0)");
    phidget_ok(Phidget_close(reinterpret_cast<PhidgetHandle>(stepper_)), "close");
    phidget_ok(PhidgetStepper_delete(&stepper_), "delete");
    stepper_ = nullptr;
    stepper_attached_ = false;
  }
}

// ============================================================================
// REAL HARDWARE: CONFIGURE PHASE
// ============================================================================
hardware_interface::CallbackReturn Science2026System::on_configure(
  const rclcpp_lifecycle::State &)
{
  // REAL HARDWARE #1: ESP32 (Shoulder Motor + Elbow Servo)
  open_esp32_serial();

  // REAL HARDWARE #2: Phidget Stepper Motor (base_yaw)
  if (!phidget_ok(PhidgetStepper_create(&stepper_), "create"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setDeviceSerialNumber(
          reinterpret_cast<PhidgetHandle>(stepper_), device_serial_),
        "setDeviceSerialNumber"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setHubPort(
          reinterpret_cast<PhidgetHandle>(stepper_), hub_port_),
        "setHubPort"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setChannel(
          reinterpret_cast<PhidgetHandle>(stepper_), channel_),
        "setChannel"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_openWaitForAttachment(
          reinterpret_cast<PhidgetHandle>(stepper_), 5000),
        "openWaitForAttachment"))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Science2026System"),
      "Stepper not attached on hub port %d. Continuing in fake mode.",
      hub_port_);

    stepper_attached_ = false;
    if (stepper_ != nullptr)
    {
      PhidgetStepper_delete(&stepper_);
      stepper_ = nullptr;
    }
  }
  else
  {
    if (!phidget_ok(
          PhidgetStepper_setRescaleFactor(stepper_, rescale_factor_deg_),
          "setRescaleFactor"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    stepper_attached_ = true;
    
    RCLCPP_INFO(
      rclcpp::get_logger("Science2026System"),
      "Stepper motor connected on serial %d, hub port %d",
      device_serial_,
      hub_port_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// REAL HARDWARE: ACTIVATE PHASE
// ============================================================================
hardware_interface::CallbackReturn Science2026System::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (esp32_serial_fd_ < 0)
  {
    open_esp32_serial();
  }

  // ========================================================================
  // REAL HARDWARE: Stepper Motor (base_yaw)
  // ========================================================================
  if (stepper_ && stepper_attached_)
  {
    if (!phidget_ok(
          PhidgetStepper_setAcceleration(stepper_, acceleration_deg_),
          "setAcceleration"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setVelocityLimit(stepper_, velocity_limit_deg_),
          "setVelocityLimit"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setCurrentLimit(stepper_, MAX_CURRENT_A),
          "setCurrentLimit"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setEngaged(stepper_, 1),
          "setEngaged"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    double pos_deg = 0.0;
    if (!phidget_ok(
          PhidgetStepper_getPosition(stepper_, &pos_deg),
          "getPosition"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_[BASE_IDX] = pos_deg * M_PI / 180.0;
    hw_commands_[BASE_IDX] = hw_states_[BASE_IDX];

    if (!phidget_ok(
      PhidgetStepper_setTargetPosition(stepper_, pos_deg),
      "setTargetPosition"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    hw_states_[BASE_IDX] = hw_commands_[BASE_IDX];
  }

  // FAKE MOTORS: Initialize all other joints to 0
  for (std::size_t i = 1; i < NUM_JOINTS; ++i)
  {
    hw_states_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  // Send initial packet: shoulder stop, servo center (90°)
  send_esp32_packet(ACT_STOP, 90);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// REAL HARDWARE: DEACTIVATE PHASE
// ============================================================================
hardware_interface::CallbackReturn Science2026System::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  send_esp32_packet(ACT_STOP, 90);
  cleanup_phidgets();
  close_esp32_serial();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// READ PHASE
// ============================================================================
hardware_interface::return_type Science2026System::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt =
    std::clamp(period.seconds(), 0.0, 0.1);

  // Base position from the Phidget
  if (stepper_ && stepper_attached_)
  {
    double position_deg = 0.0;

    if (phidget_ok(
          PhidgetStepper_getPosition(
            stepper_,
            &position_deg),
          "getPosition"))
    {
      hw_states_[BASE_IDX] =
        position_deg * M_PI / 180.0;
    }
  }
  else
  {
    hw_states_[BASE_IDX] =
      hw_commands_[BASE_IDX];
  }

  // Estimate shoulder position because there is no encoder yet
  const double shoulder_error =
    hw_commands_[SHOULDER_IDX] -
    hw_states_[SHOULDER_IDX];

  const double shoulder_max_step =
    0.45 * dt;

  hw_states_[SHOULDER_IDX] +=
    std::clamp(
      shoulder_error,
      -shoulder_max_step,
      shoulder_max_step);

  hw_states_[SHOULDER_IDX] =
    clampf(
      hw_states_[SHOULDER_IDX],
      -1.57,
      1.57);

  // Convert the last servo angle back to joint radians
  hw_states_[ELBOW_IDX] =
    (servo_current_angle_deg_ - 90.0) *
    M_PI / 180.0;

  // Joints without physical feedback
  hw_states_[WRIST_ROLL_IDX] =
    hw_commands_[WRIST_ROLL_IDX];

  hw_states_[WRIST_TWIST_IDX] =
    hw_commands_[WRIST_TWIST_IDX];

  hw_states_[CLAW_IDX] =
    hw_commands_[CLAW_IDX];

  return hardware_interface::return_type::OK;
}

// ============================================================================
// WRITE PHASE
// ============================================================================
hardware_interface::return_type Science2026System::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Clamp all commanded joint positions
  hw_commands_[BASE_IDX] =
    clampf(hw_commands_[BASE_IDX], -3.14, 3.14);

  hw_commands_[SHOULDER_IDX] =
    clampf(hw_commands_[SHOULDER_IDX], -1.57, 1.57);

  hw_commands_[ELBOW_IDX] =
    clampf(hw_commands_[ELBOW_IDX], -1.57, 1.57);

  hw_commands_[WRIST_ROLL_IDX] =
    clampf(hw_commands_[WRIST_ROLL_IDX], -1.57, 1.57);

  hw_commands_[WRIST_TWIST_IDX] =
    clampf(hw_commands_[WRIST_TWIST_IDX], -1.57, 1.57);

  hw_commands_[CLAW_IDX] =
    clampf(hw_commands_[CLAW_IDX], -1.57, 1.57);

  // Base stepper position target
  if (stepper_ && stepper_attached_)
  {
    const double base_target_deg =
      hw_commands_[BASE_IDX] * 180.0 / M_PI;

    phidget_ok(
      PhidgetStepper_setTargetPosition(
        stepper_,
        base_target_deg),
      "setTargetPosition");

    int engaged = 0;

    if (phidget_ok(
          PhidgetStepper_getEngaged(
            stepper_,
            &engaged),
          "getEngaged") &&
        !engaged)
    {
      phidget_ok(
        PhidgetStepper_setEngaged(
          stepper_,
          1),
        "setEngaged re-engage");
    }
  }

  // Shoulder actuator moves toward the requested position
  const double shoulder_error =
    hw_commands_[SHOULDER_IDX] -
    hw_states_[SHOULDER_IDX];

  const uint8_t shoulder_cmd =
    command_to_byte(shoulder_error);

  // Elbow position maps from [-1.57, 1.57] rad to [0, 180] degrees
  const double servo_angle_deg =
    angle_to_servo_byte(
      hw_commands_[ELBOW_IDX]);

  servo_current_angle_deg_ =
    servo_angle_deg;

  if (esp32_serial_fd_ >= 0)
  {
    send_esp32_packet(
      shoulder_cmd,
      static_cast<uint8_t>(
        std::lround(servo_angle_deg)));
  }

  return hardware_interface::return_type::OK;
}

}  // namespace science_2026

PLUGINLIB_EXPORT_CLASS(
  science_2026::Science2026System,
  hardware_interface::SystemInterface)
