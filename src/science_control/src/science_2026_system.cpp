#include "science_control/science_2026_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace science_2026
{

namespace
{

constexpr double PI = 3.14159265358979323846;

}  // namespace

// =============================================================================
// DESTRUCTOR
// =============================================================================

Science2026System::~Science2026System()
{
  // Stop the actuator and preserve the most recently selected tray position.
  if (esp32_serial_fd_ >= 0)
  {
    send_esp32_packet(ACT_STOP, last_tray_servo_byte_);
  }

  cleanup_phidgets();
  close_esp32_serial();
}

// =============================================================================
// INITIALIZATION
// =============================================================================

hardware_interface::CallbackReturn Science2026System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
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
      "Expected %zu joints, but received %zu.",
      NUM_JOINTS,
      joint_names_.size());

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    joint_names_[LIFT_IDX] != LIFT_JOINT_NAME ||
    joint_names_[AUGER_IDX] != AUGER_JOINT_NAME ||
    joint_names_[TRAY_IDX] != TRAY_JOINT_NAME)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "Joint order mismatch. Expected [%s, %s, %s], but received [%s, %s, %s].",
      LIFT_JOINT_NAME,
      AUGER_JOINT_NAME,
      TRAY_JOINT_NAME,
      joint_names_[LIFT_IDX].c_str(),
      joint_names_[AUGER_IDX].c_str(),
      joint_names_[TRAY_IDX].c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }

  // Verify the interfaces declared in the URDF.
  const auto & lift_joint = info_.joints[LIFT_IDX];
  const auto & auger_joint = info_.joints[AUGER_IDX];
  const auto & tray_joint = info_.joints[TRAY_IDX];

  if (
    lift_joint.command_interfaces.size() != 1 ||
    lift_joint.command_interfaces[0].name !=
    hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "%s must have exactly one velocity command interface.",
      LIFT_JOINT_NAME);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    auger_joint.command_interfaces.size() != 1 ||
    auger_joint.command_interfaces[0].name !=
    hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "%s must have exactly one velocity command interface.",
      AUGER_JOINT_NAME);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    tray_joint.command_interfaces.size() != 1 ||
    tray_joint.command_interfaces[0].name !=
    hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "%s must have exactly one position command interface.",
      TRAY_JOINT_NAME);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    lift_joint.state_interfaces.size() != 1 ||
    lift_joint.state_interfaces[0].name !=
    hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "%s must have exactly one position state interface.",
      LIFT_JOINT_NAME);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    auger_joint.state_interfaces.size() != 1 ||
    auger_joint.state_interfaces[0].name !=
    hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "%s must have exactly one position state interface.",
      AUGER_JOINT_NAME);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    tray_joint.state_interfaces.size() != 1 ||
    tray_joint.state_interfaces[0].name !=
    hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "%s must have exactly one position state interface.",
      TRAY_JOINT_NAME);

    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read optional hardware parameters from the URDF.
  const auto serial_device_it =
    info_.hardware_parameters.find("esp32_serial_device");

  if (serial_device_it != info_.hardware_parameters.end())
  {
    esp32_serial_device_ = serial_device_it->second;
  }

  const auto phidget_serial_it =
    info_.hardware_parameters.find("phidget_serial");

  if (phidget_serial_it != info_.hardware_parameters.end())
  {
    try
    {
      device_serial_ = std::stoi(phidget_serial_it->second);
    }
    catch (const std::exception & exception)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("Science2026System"),
        "Invalid phidget_serial value '%s': %s",
        phidget_serial_it->second.c_str(),
        exception.what());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto hub_port_it =
    info_.hardware_parameters.find("phidget_hub_port");

  if (hub_port_it != info_.hardware_parameters.end())
  {
    try
    {
      hub_port_ = std::stoi(hub_port_it->second);
    }
    catch (const std::exception & exception)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("Science2026System"),
        "Invalid phidget_hub_port value '%s': %s",
        hub_port_it->second.c_str(),
        exception.what());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto channel_it =
    info_.hardware_parameters.find("phidget_channel");

  if (channel_it != info_.hardware_parameters.end())
  {
    try
    {
      channel_ = std::stoi(channel_it->second);
    }
    catch (const std::exception & exception)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("Science2026System"),
        "Invalid phidget_channel value '%s': %s",
        channel_it->second.c_str(),
        exception.what());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  lift_velocity_command_ = 0.0;
  auger_velocity_command_ = 0.0;
  tray_position_command_ = TRAY_POSITION_0_RAD;

  lift_position_state_ = 0.0;
  auger_position_state_ = 0.0;
  tray_position_state_ = TRAY_POSITION_0_RAD;

  last_lift_command_byte_ = ACT_STOP;
  last_tray_servo_byte_ = 0;
  last_auger_velocity_command_ = 0.0;
  auger_target_position_rad_ = 0.0;

  hardware_active_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("Science2026System"),
    "Science hardware interface initialized with three joints.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// STATE INTERFACES
// =============================================================================

std::vector<hardware_interface::StateInterface>
Science2026System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.reserve(3);

  state_interfaces.emplace_back(
    LIFT_JOINT_NAME,
    hardware_interface::HW_IF_POSITION,
    &lift_position_state_);

  state_interfaces.emplace_back(
    AUGER_JOINT_NAME,
    hardware_interface::HW_IF_POSITION,
    &auger_position_state_);

  state_interfaces.emplace_back(
    TRAY_JOINT_NAME,
    hardware_interface::HW_IF_POSITION,
    &tray_position_state_);

  return state_interfaces;
}

// =============================================================================
// COMMAND INTERFACES
// =============================================================================

std::vector<hardware_interface::CommandInterface>
Science2026System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.reserve(3);

  command_interfaces.emplace_back(
    LIFT_JOINT_NAME,
    hardware_interface::HW_IF_VELOCITY,
    &lift_velocity_command_);

  command_interfaces.emplace_back(
    AUGER_JOINT_NAME,
    hardware_interface::HW_IF_VELOCITY,
    &auger_velocity_command_);

  command_interfaces.emplace_back(
    TRAY_JOINT_NAME,
    hardware_interface::HW_IF_POSITION,
    &tray_position_command_);

  return command_interfaces;
}

// =============================================================================
// GENERAL HELPERS
// =============================================================================

double Science2026System::clamp_value(
  double value,
  double minimum,
  double maximum) const
{
  return std::clamp(value, minimum, maximum);
}

bool Science2026System::phidget_ok(
  PhidgetReturnCode code,
  const char * context) const
{
  if (code == EPHIDGET_OK)
  {
    return true;
  }

  const char * error_description = "Unknown Phidget error";
  Phidget_getErrorDescription(code, &error_description);

  RCLCPP_ERROR(
    rclcpp::get_logger("Science2026System"),
    "Phidget error in %s: %s",
    context,
    error_description);

  return false;
}

// =============================================================================
// ESP32 SERIAL
// =============================================================================

bool Science2026System::open_esp32_serial()
{
  close_esp32_serial();

  constexpr int MAX_ATTEMPTS = 3;

  for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt)
  {
    esp32_serial_fd_ = ::open(
      esp32_serial_device_.c_str(),
      O_RDWR | O_NOCTTY | O_SYNC);

    if (esp32_serial_fd_ >= 0)
    {
      break;
    }

    if (attempt < MAX_ATTEMPTS)
    {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(500));
    }
  }

  if (esp32_serial_fd_ < 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Science2026System"),
      "ESP32 was not found on %s. Continuing without ESP32 hardware.",
      esp32_serial_device_.c_str());

    esp32_connected_ = false;
    return false;
  }

  termios tty{};

  if (tcgetattr(esp32_serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "tcgetattr failed for %s: %s",
      esp32_serial_device_.c_str(),
      std::strerror(errno));

    close_esp32_serial();
    return false;
  }

  cfmakeraw(&tty);

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(esp32_serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Science2026System"),
      "tcsetattr failed for %s: %s",
      esp32_serial_device_.c_str(),
      std::strerror(errno));

    close_esp32_serial();
    return false;
  }

  tcflush(esp32_serial_fd_, TCIOFLUSH);

  esp32_connected_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("Science2026System"),
    "ESP32 connected on %s at 115200 baud.",
    esp32_serial_device_.c_str());

  return true;
}

void Science2026System::close_esp32_serial()
{
  if (esp32_serial_fd_ >= 0)
  {
    ::close(esp32_serial_fd_);
    esp32_serial_fd_ = -1;
  }

  esp32_connected_ = false;
}

bool Science2026System::send_esp32_packet(
  uint8_t lift_command,
  uint8_t tray_angle_degrees)
{
  if (esp32_serial_fd_ < 0)
  {
    return false;
  }

  const uint8_t header = 0xAA;
  const uint8_t checksum =
    static_cast<uint8_t>(lift_command ^ tray_angle_degrees);

  const uint8_t packet[4] = {
    header,
    lift_command,
    tray_angle_degrees,
    checksum
  };

  std::size_t total_written = 0;

  while (total_written < sizeof(packet))
  {
    const ssize_t bytes_written = ::write(
      esp32_serial_fd_,
      packet + total_written,
      sizeof(packet) - total_written);

    if (bytes_written > 0)
    {
      total_written += static_cast<std::size_t>(bytes_written);
      continue;
    }

    if (bytes_written < 0 && errno == EINTR)
    {
      continue;
    }

    RCLCPP_WARN(
      rclcpp::get_logger("Science2026System"),
      "Failed to write a complete packet to the ESP32: %s",
      std::strerror(errno));

    return false;
  }

  return true;
}

// =============================================================================
// LINEAR ACTUATOR HELPERS
// =============================================================================

uint8_t Science2026System::lift_command_to_byte(
  double velocity_command) const
{
  if (!std::isfinite(velocity_command))
  {
    return ACT_STOP;
  }

  if (velocity_command > actuator_command_deadband_)
  {
    return ACT_EXTEND;
  }

  if (velocity_command < -actuator_command_deadband_)
  {
    return ACT_RETRACT;
  }

  return ACT_STOP;
}

// =============================================================================
// BEAKER TRAY HELPERS
// =============================================================================

double Science2026System::snap_tray_angle_radians(
  double angle_radians) const
{
  if (!std::isfinite(angle_radians))
  {
    return TRAY_POSITION_0_RAD;
  }

  const double distance_to_zero =
    std::fabs(angle_radians - TRAY_POSITION_0_RAD);

  const double distance_to_ninety =
    std::fabs(angle_radians - TRAY_POSITION_90_RAD);

  const double distance_to_one_eighty =
    std::fabs(angle_radians - TRAY_POSITION_180_RAD);

  if (
    distance_to_zero <= distance_to_ninety &&
    distance_to_zero <= distance_to_one_eighty)
  {
    return TRAY_POSITION_0_RAD;
  }

  if (distance_to_ninety <= distance_to_one_eighty)
  {
    return TRAY_POSITION_90_RAD;
  }

  return TRAY_POSITION_180_RAD;
}

uint8_t Science2026System::tray_radians_to_servo_byte(
  double angle_radians) const
{
  const double snapped_angle =
    snap_tray_angle_radians(angle_radians);

  const double angle_degrees =
    snapped_angle * 180.0 / PI;

  return static_cast<uint8_t>(
    std::lround(
      clamp_value(angle_degrees, 0.0, 180.0)));
}

// =============================================================================
// PHIDGET CLEANUP
// =============================================================================

void Science2026System::cleanup_phidgets()
{
  if (stepper_ == nullptr)
  {
    stepper_attached_ = false;
    return;
  }

  if (stepper_attached_)
  {
    phidget_ok(
      PhidgetStepper_setEngaged(stepper_, 0),
      "setEngaged(0)");
  }

  phidget_ok(
    Phidget_close(
      reinterpret_cast<PhidgetHandle>(stepper_)),
    "close");

  phidget_ok(
    PhidgetStepper_delete(&stepper_),
    "delete");

  stepper_ = nullptr;
  stepper_attached_ = false;
}

// =============================================================================
// CONFIGURE
// =============================================================================

hardware_interface::CallbackReturn Science2026System::on_configure(
  const rclcpp_lifecycle::State &)
{
  hardware_active_ = false;

  // ESP32 is optional during development.
  open_esp32_serial();

  // Phidget is also optional during development.
  cleanup_phidgets();

  if (!phidget_ok(
      PhidgetStepper_create(&stepper_),
      "PhidgetStepper_create"))
  {
    stepper_ = nullptr;

    RCLCPP_WARN(
      rclcpp::get_logger("Science2026System"),
      "Could not create the Phidget stepper handle. Continuing in fake mode.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!phidget_ok(
      Phidget_setDeviceSerialNumber(
        reinterpret_cast<PhidgetHandle>(stepper_),
        device_serial_),
      "setDeviceSerialNumber"))
  {
    cleanup_phidgets();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
      Phidget_setHubPort(
        reinterpret_cast<PhidgetHandle>(stepper_),
        hub_port_),
      "setHubPort"))
  {
    cleanup_phidgets();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
      Phidget_setChannel(
        reinterpret_cast<PhidgetHandle>(stepper_),
        channel_),
      "setChannel"))
  {
    cleanup_phidgets();
    return hardware_interface::CallbackReturn::ERROR;
  }

  const PhidgetReturnCode attachment_result =
    Phidget_openWaitForAttachment(
      reinterpret_cast<PhidgetHandle>(stepper_),
      5000);

  if (attachment_result != EPHIDGET_OK)
  {
    const char * error_description = "Unknown Phidget error";
    Phidget_getErrorDescription(
      attachment_result,
      &error_description);

    RCLCPP_WARN(
      rclcpp::get_logger("Science2026System"),
      "Phidget stepper did not attach on serial %d, hub port %d, channel %d: %s. "
      "Continuing in fake mode.",
      device_serial_,
      hub_port_,
      channel_,
      error_description);

    cleanup_phidgets();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  stepper_attached_ = true;

  if (!phidget_ok(
      PhidgetStepper_setRescaleFactor(
        stepper_,
        auger_rescale_factor_deg_),
      "setRescaleFactor"))
  {
    cleanup_phidgets();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Science2026System"),
    "Phidget auger stepper attached on serial %d, hub port %d, channel %d.",
    device_serial_,
    hub_port_,
    channel_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// ACTIVATE
// =============================================================================

hardware_interface::CallbackReturn Science2026System::on_activate(
  const rclcpp_lifecycle::State &)
{
  lift_velocity_command_ = 0.0;
  auger_velocity_command_ = 0.0;

  tray_position_command_ =
    snap_tray_angle_radians(tray_position_command_);

  last_lift_command_byte_ = ACT_STOP;
  last_tray_servo_byte_ =
    tray_radians_to_servo_byte(tray_position_command_);

  last_auger_velocity_command_ = 0.0;

  if (esp32_serial_fd_ < 0)
  {
    open_esp32_serial();
  }

  if (esp32_connected_)
  {
    send_esp32_packet(
      ACT_STOP,
      last_tray_servo_byte_);
  }

  if (stepper_ != nullptr && stepper_attached_)
  {
    // Match the arm Phidget initialization exactly.
    if (!phidget_ok(
        PhidgetStepper_setAcceleration(
          stepper_,
          auger_acceleration_deg_),
        "PhidgetStepper_setAcceleration auger"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
        PhidgetStepper_setVelocityLimit(
          stepper_,
          auger_velocity_limit_deg_),
        "PhidgetStepper_setVelocityLimit auger"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
        PhidgetStepper_setCurrentLimit(
          stepper_,
          auger_current_limit_a_),
        "PhidgetStepper_setCurrentLimit auger"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
        PhidgetStepper_setEngaged(
          stepper_,
          1),
        "PhidgetStepper_setEngaged auger"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    double auger_position_deg = 0.0;

    if (!phidget_ok(
        PhidgetStepper_getPosition(
          stepper_,
          &auger_position_deg),
        "PhidgetStepper_getPosition auger"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    auger_target_position_rad_ =
      auger_position_deg * PI / 180.0;

    auger_position_state_ =
      auger_target_position_rad_;

    phidget_ok(
      PhidgetStepper_setTargetPosition(
        stepper_,
        auger_position_deg),
      "PhidgetStepper_setTargetPosition auger");
  }
  else
  {
    auger_target_position_rad_ =
      auger_position_state_;
  }

  hardware_active_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("Science2026System"),
    "Science hardware interface activated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// DEACTIVATE
// =============================================================================

hardware_interface::CallbackReturn Science2026System::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  hardware_active_ = false;

  lift_velocity_command_ = 0.0;
  auger_velocity_command_ = 0.0;

  if (esp32_serial_fd_ >= 0)
  {
    send_esp32_packet(
      ACT_STOP,
      tray_radians_to_servo_byte(tray_position_command_));
  }

  if (stepper_ != nullptr && stepper_attached_)
  {
    phidget_ok(
      PhidgetStepper_setEngaged(
        stepper_,
        0),
      "PhidgetStepper_setEngaged(0) auger");
  }

  cleanup_phidgets();
  close_esp32_serial();

  last_lift_command_byte_ = ACT_STOP;
  last_auger_velocity_command_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("Science2026System"),
    "Science hardware interface deactivated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// READ
// =============================================================================

hardware_interface::return_type Science2026System::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt =
    clamp_value(period.seconds(), 0.0, 0.1);

  /*
   * Linear actuator:
   * There is no position feedback, so estimate motion only for RViz.
   *
   * The velocity command is expected to be approximately:
   *   +1.0 = extend
   *   -1.0 = retract
   *    0.0 = stop
   */
  double normalized_lift_command = 0.0;

  if (std::isfinite(lift_velocity_command_))
  {
    normalized_lift_command =
      clamp_value(lift_velocity_command_, -1.0, 1.0);
  }

  lift_position_state_ +=
    normalized_lift_command *
    estimated_lift_speed_m_s_ *
    dt;

  lift_position_state_ = clamp_value(
    lift_position_state_,
    estimated_lift_min_position_m_,
    estimated_lift_max_position_m_);

  /*
   * Auger position state. The Phidget position is expressed in degrees after
   * applying the same rescale factor used by the arm. Convert it back to ROS
   * radians. Without hardware, report the integrated target position.
   */
  if (stepper_ != nullptr && stepper_attached_)
  {
    double auger_position_deg = 0.0;

    if (phidget_ok(
        PhidgetStepper_getPosition(
          stepper_,
          &auger_position_deg),
        "PhidgetStepper_getPosition auger"))
    {
      auger_position_state_ =
        auger_position_deg * PI / 180.0;
    }
  }
  else
  {
    auger_position_state_ =
      auger_target_position_rad_;
  }

  /*
   * Tray:
   * There is no servo feedback, so report the snapped commanded position.
   */
  tray_position_state_ =
    snap_tray_angle_radians(tray_position_command_);

  return hardware_interface::return_type::OK;
}

// =============================================================================
// WRITE
// =============================================================================

hardware_interface::return_type Science2026System::write(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!hardware_active_)
  {
    return hardware_interface::return_type::OK;
  }

  // -------------------------------------------------------------------------
  // Linear actuator command
  // -------------------------------------------------------------------------

  if (!std::isfinite(lift_velocity_command_))
  {
    lift_velocity_command_ = 0.0;
  }

  lift_velocity_command_ =
    clamp_value(lift_velocity_command_, -1.0, 1.0);

  const uint8_t lift_command_byte =
    lift_command_to_byte(lift_velocity_command_);

  // -------------------------------------------------------------------------
  // Beaker tray command
  // -------------------------------------------------------------------------

  tray_position_command_ =
    snap_tray_angle_radians(tray_position_command_);

  const uint8_t tray_servo_byte =
    tray_radians_to_servo_byte(tray_position_command_);

  /*
   * Send both ESP32 commands together.
   *
   * The packet is sent every control cycle so the ESP32 always receives the
   * current actuator state. A firmware-side timeout is still recommended.
   */
  if (esp32_connected_)
  {
    if (!send_esp32_packet(
        lift_command_byte,
        tray_servo_byte))
    {
      RCLCPP_WARN(
        rclcpp::get_logger("Science2026System"),
        "Failed to send command packet to the ESP32.");
    }
  }

  last_lift_command_byte_ = lift_command_byte;
  last_tray_servo_byte_ = tray_servo_byte;

  // -------------------------------------------------------------------------
  // Auger stepper command
  // -------------------------------------------------------------------------

  const double dt =
    clamp_value(period.seconds(), 0.0, 0.1);

  if (!std::isfinite(auger_velocity_command_))
  {
    auger_velocity_command_ = 0.0;
  }

  auger_velocity_command_ =
    clamp_value(
      auger_velocity_command_,
      -auger_command_limit_rad_s_,
      auger_command_limit_rad_s_);

  // Same method as the arm: integrate velocity into a position target.
  auger_target_position_rad_ +=
    auger_velocity_command_ * dt;

  const double auger_target_position_deg =
    auger_target_position_rad_ * 180.0 / PI;

  if (stepper_ != nullptr && stepper_attached_)
  {
    int engaged = 0;

    if (
      phidget_ok(
        PhidgetStepper_getEngaged(
          stepper_,
          &engaged),
        "PhidgetStepper_getEngaged auger") &&
      !engaged)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("Science2026System"),
        "Auger stepper was disengaged. Re-engaging.");

      if (!phidget_ok(
          PhidgetStepper_setEngaged(
            stepper_,
            1),
          "PhidgetStepper_setEngaged auger re-engage"))
      {
        return hardware_interface::return_type::ERROR;
      }
    }

    if (!phidget_ok(
        PhidgetStepper_setTargetPosition(
          stepper_,
          auger_target_position_deg),
        "PhidgetStepper_setTargetPosition auger"))
    {
      return hardware_interface::return_type::ERROR;
    }
  }

  last_auger_velocity_command_ =
    auger_velocity_command_;

  return hardware_interface::return_type::OK;
}

}  // namespace science_2026

PLUGINLIB_EXPORT_CLASS(
  science_2026::Science2026System,
  hardware_interface::SystemInterface)
