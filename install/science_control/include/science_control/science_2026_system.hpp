#ifndef SCIENCE_2026__SCIENCE_2026_SYSTEM_HPP_
#define SCIENCE_2026__SCIENCE_2026_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <phidget22.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace science_2026
{

class Science2026System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Science2026System)

  ~Science2026System() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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
  // ========================================================================
  // HELPER FUNCTIONS
  // ========================================================================
  void cleanup_phidgets();
  bool phidget_ok(PhidgetReturnCode code, const char * context) const;
  double clampf(double x, double lo, double hi) const;

  // ========================================================================
  // REAL HARDWARE: ESP32 (Shoulder Motor + Elbow Servo)
  // ONE serial port for BOTH
  // ========================================================================
  bool open_esp32_serial();
  void close_esp32_serial();
  bool send_esp32_packet(uint8_t shoulder_cmd, uint8_t servo_angle);
  uint8_t command_to_byte(double value) const;
  double angle_to_servo_byte(double angle_rad) const;

  // ========================================================================
  // JOINT CONSTANTS
  // ========================================================================
  static constexpr std::size_t NUM_JOINTS = 6;
  static constexpr std::size_t BASE_IDX = 0;       // Phidget stepper
  static constexpr std::size_t SHOULDER_IDX = 1;   // Linear actuator (ESP32)
  static constexpr std::size_t ELBOW_IDX = 2;      // SERVO (ESP32)
  static constexpr std::size_t WRIST_ROLL_IDX = 3;
  static constexpr std::size_t WRIST_TWIST_IDX = 4;
  static constexpr std::size_t CLAW_IDX = 5;

  // ========================================================================
  // ACTUATOR COMMANDS
  // ========================================================================
  static constexpr uint8_t ACT_STOP = 0;
  static constexpr uint8_t ACT_EXTEND = 1;
  static constexpr uint8_t ACT_RETRACT = 2;

  // ========================================================================
  // STEPPER MOTOR LIMITS (NEMA11 100:1 Gearbox - Phidgets 3322_1)
  // ========================================================================
  static constexpr double MAX_VELOCITY_RAD_S = 1.5;
  static constexpr double MAX_CURRENT_A = 0.67;

  // ========================================================================
  // JOINT STATE/COMMAND STORAGE
  // ========================================================================
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<std::string> joint_names_;

  // ========================================================================
  // REAL HARDWARE: ESP32 Serial (Shoulder + Servo)
  // ========================================================================
  int esp32_serial_fd_ = -1;
  std::string esp32_serial_device_ = "/dev/ttyUSB0";
  double actuator_command_deadband_ = 0.2;

  // Servo state tracking
  double servo_target_angle_deg_ = 90.0;
  double servo_current_angle_deg_ = 90.0;
  double servo_min_angle_deg_ = 0.0;
  double servo_max_angle_deg_ = 180.0;

  // ========================================================================
  // REAL HARDWARE: Phidget Stepper Motor (base_yaw)
  // ========================================================================
  PhidgetStepperHandle stepper_ = nullptr;
  bool stepper_attached_ = false;

  int device_serial_ = 766944;
  int hub_port_ = 4;
  int channel_ = 0;
  
  double rescale_factor_deg_ = 0.00146103896;
  double velocity_limit_deg_ = 86.0;
  double acceleration_deg_ = 200.0;
};

}  // namespace science_2026

#endif  // SCIENCE_2026__SCIENCE_2026_SYSTEM_HPP_