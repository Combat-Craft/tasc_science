#ifndef SCIENCE_CONTROL__SCIENCE_2026_SYSTEM_HPP_
#define SCIENCE_CONTROL__SCIENCE_2026_SYSTEM_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <phidget22.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace science_2026
{

class Science2026System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Science2026System)

  Science2026System() = default;
  ~Science2026System() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // =========================================================================
  // JOINT DEFINITIONS
  // =========================================================================

  static constexpr std::size_t NUM_JOINTS = 3;

  static constexpr std::size_t LIFT_IDX = 0;
  static constexpr std::size_t AUGER_IDX = 1;
  static constexpr std::size_t TRAY_IDX = 2;

  static constexpr const char * LIFT_JOINT_NAME =
    "science_lift_joint";

  static constexpr const char * AUGER_JOINT_NAME =
    "auger_spin_joint";

  static constexpr const char * TRAY_JOINT_NAME =
    "beaker_tray_joint";

  // =========================================================================
  // HELPER FUNCTIONS
  // =========================================================================

  void cleanup_phidgets();

  bool phidget_ok(
    PhidgetReturnCode code,
    const char * context) const;

  double clamp_value(
    double value,
    double minimum,
    double maximum) const;

  // =========================================================================
  // ESP32 SERIAL
  //
  // The ESP32 controls:
  //   1. Linear actuator for the auger lift
  //   2. Servo for the beaker tray
  //
  // Packet format:
  //   [0xAA] [lift command] [tray angle] [checksum]
  // =========================================================================

  bool open_esp32_serial();

  void close_esp32_serial();

  bool send_esp32_packet(
    uint8_t lift_command,
    uint8_t tray_angle_degrees);

  uint8_t lift_command_to_byte(
    double velocity_command) const;

  uint8_t tray_radians_to_servo_byte(
    double angle_radians) const;

  double snap_tray_angle_radians(
    double angle_radians) const;

  // =========================================================================
  // LINEAR ACTUATOR COMMAND BYTES
  // =========================================================================

  static constexpr uint8_t ACT_STOP = 0;
  static constexpr uint8_t ACT_EXTEND = 1;
  static constexpr uint8_t ACT_RETRACT = 2;

  // =========================================================================
  // BEAKER TRAY POSITIONS
  // =========================================================================

  static constexpr double TRAY_POSITION_0_RAD =
    0.0;

  static constexpr double TRAY_POSITION_90_RAD =
    1.5707963267948966;

  static constexpr double TRAY_POSITION_180_RAD =
    3.1415926535897932;

  // =========================================================================
  // JOINT NAMES
  // =========================================================================

  std::vector<std::string> joint_names_;

  // =========================================================================
  // COMMAND INTERFACES
  //
  // science_lift_joint:
  //   velocity command
  //   positive = extend
  //   negative = retract
  //   zero = stop
  //
  // auger_spin_joint:
  //   velocity command
  //   positive = forward digging direction
  //   negative = reverse digging direction
  //   zero = stop
  //
  // beaker_tray_joint:
  //   position command
  //   valid positions are 0, 90, and 180 degrees
  // =========================================================================

  double lift_velocity_command_ = 0.0;

  double auger_velocity_command_ = 0.0;

  double tray_position_command_ = TRAY_POSITION_0_RAD;

  // =========================================================================
  // STATE INTERFACES
  //
  // There is currently no physical feedback.
  //
  // The lift position is estimated for RViz.
  // The auger position is integrated from the commanded velocity for RViz.
  // The tray position is assumed to equal the commanded servo position.
  // =========================================================================

  double lift_position_state_ = 0.0;

  double auger_position_state_ = 0.0;

  double tray_position_state_ = TRAY_POSITION_0_RAD;

  // =========================================================================
  // ESP32 SERIAL CONFIGURATION
  // =========================================================================

  int esp32_serial_fd_ = -1;

  std::string esp32_serial_device_ = "/dev/ttyUSB0";

  bool esp32_connected_ = false;

  double actuator_command_deadband_ = 0.2;

  // Estimated actuator speed used only for RViz visualization.
  double estimated_lift_speed_m_s_ = 0.03;

  // Estimated RViz position bounds.
  // These do not limit the physical actuator.
  double estimated_lift_min_position_m_ = -1.0;
  double estimated_lift_max_position_m_ = 1.0;

  // =========================================================================
  // PHIDGET STEPPER CONFIGURATION
  //
  // The Phidget stepper continuously spins the auger.
  // The stepper is controlled using a velocity command.
  // =========================================================================

  PhidgetStepperHandle stepper_ = nullptr;

  bool stepper_attached_ = false;

  int device_serial_ = 766944;
  int hub_port_ = 4;
  int channel_ = 0;

  /*
   * Match the arm Phidget stepper configuration exactly.
   * The ROS controller supplies velocity in rad/s. The hardware interface
   * integrates that command into a position target, converts it to degrees,
   * and sends it to the Phidget using setTargetPosition().
   */
  double auger_target_position_rad_ = 0.0;
  double auger_command_limit_rad_s_ = 1.0;

  double auger_rescale_factor_deg_ = 0.001125;
  double auger_velocity_limit_deg_ = 75.0;
  double auger_acceleration_deg_ = 150.0;
  double auger_current_limit_a_ = 0.67;

  // =========================================================================
  // LAST HARDWARE COMMANDS
  // =========================================================================

  uint8_t last_lift_command_byte_ = ACT_STOP;

  uint8_t last_tray_servo_byte_ = 0;

  double last_auger_velocity_command_ = 0.0;

  // =========================================================================
  // HARDWARE STATUS
  // =========================================================================

  bool hardware_active_ = false;
};

}  // namespace science_2026

#endif  // SCIENCE_CONTROL__SCIENCE_2026_SYSTEM_HPP_
